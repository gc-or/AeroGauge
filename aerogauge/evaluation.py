"""Fixed-seed evaluation, independent of training rewards."""
import csv
import hashlib
import json
import platform
import sys
import time
from contextlib import nullcontext
from pathlib import Path

import mujoco
import numpy as np
import scipy

from . import __version__
from .tasks import reference


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+"\n",
                          encoding="utf-8")


def failure_reason(state, target, contact):
    if contact:
        return "ground_contact"
    if state["position"][2] < .2 or state["position"][2] > 5:
        return "height_limit"
    if np.linalg.norm(state["position"]-target) > 3:
        return "position_limit"
    if state["rotation"][2, 2] < .5:
        return "tilt_limit"
    return None


def summarize(rows, reason, config):
    error = np.array([r["position_error_m"] for r in rows])
    tail = [r for r in rows if r["time_s"] > config["duration_s"]-2.]
    complete = rows[-1]["time_s"] >= config["duration_s"]-1e-9
    success = bool(complete and reason is None and tail and all(
        r["position_error_m"] < .1 and r["velocity_error_m_s"] < .1
        and r["tilt_deg"] < 10 and r["angular_speed_rad_s"] < .3 for r in tail))
    return {"success": success, "completed": complete,
            "termination": reason or ("completed" if success else "tracking_threshold"),
            "elapsed_s": rows[-1]["time_s"], "samples": len(rows),
            "position_rmse_m": float(np.sqrt(np.mean(error**2))),
            "position_max_error_m": float(error.max()),
            "velocity_rmse_m_s": float(np.sqrt(np.mean([r["velocity_error_m_s"]**2 for r in rows]))),
            "motor_limit_fraction": float(np.mean([r["at_motor_limit"] for r in rows]))}


def evaluate(sim, policy, task, seeds, out, viewer=False):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    write_json(out / "config.json", sim.config)
    root = Path(__file__).parent
    hashes = {str(p.relative_to(root)).replace("\\", "/"): digest(p)
              for p in sorted(root.rglob("*")) if p.is_file() and p.suffix in (".py", ".json", ".xml", ".obj")}
    write_json(out / "manifest.json", {
        "aerogauge": __version__, "python": sys.version, "platform": platform.platform(),
        "mujoco": mujoco.__version__, "baseline_mujoco": "3.15.0",
        "baseline_version_match": mujoco.__version__ == "3.15.0",
        "numpy": np.__version__, "scipy": scipy.__version__, "task": task,
        "seeds": seeds, "policy": type(policy).__name__,
        "policy_sha256": getattr(policy, "sha256", None),
        "policy_contract": getattr(policy, "contract", None),
        "files_sha256": hashes,
        "success_definition": "Full episode; every control sample in final 2s: position error <0.1m, velocity error <0.1m/s, tilt <10deg, angular speed <0.3rad/s; no failure.",
        "limits": "Synthetic parameter sweeps, truth-state input, proxy motors. No real-world validation or training checkpoint validation implied."})
    if viewer:
        from mujoco import viewer as viewer_module
        context = viewer_module.launch_passive(sim.model, sim.data)
    else:
        context = nullcontext(None)
    episodes = []
    with context as view:
        for index, seed in enumerate(seeds):
            s = sim.reset(seed)
            policy.reset()
            write_json(out / f"initial_{index:03d}.json", {k: v.tolist() if isinstance(v, np.ndarray) else v
                                                        for k, v in s.items()})
            rows, reason = [], None
            for _ in range(round(sim.config["duration_s"]/sim.control_dt)):
                started = time.monotonic()
                target, velocity = reference(task, s["time_s"], sim.config)
                thrust = policy.act(s, target, velocity)
                s = sim.step(thrust)
                target, velocity = reference(task, s["time_s"], sim.config)
                reason = failure_reason(s, target, sim.had_contact)
                rows.append({"time_s": s["time_s"],
                    **dict(zip(("x_m", "y_m", "z_m"), s["position"])),
                    **dict(zip(("vx_m_s", "vy_m_s", "vz_m_s"), s["velocity"])),
                    **dict(zip(("wx_rad_s", "wy_rad_s", "wz_rad_s"), s["omega"])),
                    **dict(zip(("asset_qw", "asset_qx", "asset_qy", "asset_qz"), sim.data.qpos[3:7])),
                    **dict(zip(("target_x_m", "target_y_m", "target_z_m"), target)),
                    "position_error_m": float(np.linalg.norm(s["position"]-target)),
                    "velocity_error_m_s": float(np.linalg.norm(s["velocity"]-velocity)),
                    "tilt_deg": float(np.rad2deg(np.arccos(np.clip(s["rotation"][2, 2], -1, 1)))),
                    "angular_speed_rad_s": float(np.linalg.norm(s["omega"])),
                    "at_motor_limit": int(sim.clipped),
                    **{f"motor_{i+1}_requested_N": float(thrust[i]) for i in range(4)},
                    **{f"motor_{i+1}_executed_N": float(sim.motor[i]*sim.efficiency[i]) for i in range(4)}})
                if view is not None:
                    if not view.is_running():
                        reason = "user_closed_viewer"
                    view.sync()
                    time.sleep(max(0., sim.control_dt-(time.monotonic()-started)))
                if reason:
                    break
            with (out / f"trajectory_{index:03d}.csv").open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            episodes.append({"seed": seed, **summarize(rows, reason, sim.config)})
            write_json(out / "episodes.json", episodes)
            if reason == "user_closed_viewer":
                break
    summary = {"requested_episodes": len(seeds), "evaluated_episodes": len(episodes),
               "successful_episodes": sum(e["success"] for e in episodes),
               "success_rate": sum(e["success"] for e in episodes)/len(episodes),
               "position_rmse_mean_m": float(np.mean([e["position_rmse_m"] for e in episodes])),
               "episodes": episodes}
    write_json(out / "summary.json", summary)
    return summary
