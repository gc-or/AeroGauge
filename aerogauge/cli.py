import argparse
import json
from pathlib import Path

import numpy as np

from .evaluation import evaluate
from .policies import PositionController, TorchScriptHover
from .simulation import Simulation
from .vehicle import DATA


def merge(base, patch):
    for key, value in patch.items():
        if key not in base:
            raise ValueError(f"Unknown configuration key: {key}")
        if isinstance(base[key], dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path=None):
    config = json.loads((DATA / "basic.json").read_text(encoding="utf-8"))
    if path:
        merge(config, json.loads(Path(path).read_text(encoding="utf-8")))
    def finite_values(value):
        if isinstance(value, dict):
            return all(finite_values(v) for v in value.values())
        return bool(np.isfinite(np.asarray(value, dtype=float)).all())
    if not finite_values(config):
        raise ValueError("All configuration numbers must be finite")
    for key in ("initial_position_m", "initial_position_half_range_m", "target_position_m"):
        if np.asarray(config[key]).shape != (3,):
            raise ValueError(f"{key} must have three components")
    if config["initial_tilt_deg"] < 0 or np.any(np.array(config["initial_position_half_range_m"]) < 0):
        raise ValueError("Initial perturbation ranges must be nonnegative")
    d = config["disturbance"]
    for key in ("constant_force_world_N", "pulse_force_world_N", "wind_world_m_s", "drag_cd_area_m2"):
        if np.asarray(d[key]).shape != (3,):
            raise ValueError(f"{key} must have three components")
    if d["air_density_kg_m3"] < 0 or np.any(np.array(d["drag_cd_area_m2"]) < 0):
        raise ValueError("Air density and drag coefficients must be nonnegative")
    if len(d["pulse_interval_s"]) != 2 or not 0 <= d["pulse_interval_s"][0] <= d["pulse_interval_s"][1]:
        raise ValueError("Invalid pulse interval")
    if config["plant"]["kappa_m"] <= 0:
        raise ValueError("kappa_m must be positive")
    duration, control = config["duration_s"], config["control_dt_s"]
    if (not np.isfinite([duration, control, config["physics_dt_s"]]).all()
            or control <= 0 or config["physics_dt_s"] <= 0 or duration < 2
            or not np.isclose(round(duration/control)*control, duration, rtol=0, atol=1e-9)):
        raise ValueError("Finite duration >=2s must be a multiple of positive control_dt_s")
    return config


def main():
    parser = argparse.ArgumentParser(description="WRJ MuJoCo evaluation")
    parser.add_argument("--task", choices=("hover", "step", "circle"), default="hover")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--out", type=Path, required=True, help="New output directory")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--policy", type=Path, help="Exported feed-forward TorchScript actor")
    parser.add_argument("--contract", type=Path, help="Explicit hover16 policy metadata")
    args = parser.parse_args()
    config = load_config(args.config)
    if bool(args.policy) != bool(args.contract):
        parser.error("--policy and --contract must be provided together")
    if args.policy:
        contract = json.loads(args.contract.read_text(encoding="utf-8"))
        if args.task != "hover" or not np.isclose(contract["control_dt_s"], config["control_dt_s"]):
            parser.error("hover16 adapter requires hover task and matching control period")
        if any(contract[k] <= 0 for k in ("position_scale_m", "velocity_scale_m_s",
                                          "angular_velocity_scale_rad_s", "trim_fraction_scale")):
            parser.error("Policy scales must be positive")
    sim = Simulation(config)
    policy = (TorchScriptHover(args.policy, contract, sim.vehicle) if args.policy
              else PositionController(sim.vehicle))
    result = evaluate(sim, policy, args.task, args.seeds, args.out, args.viewer)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
