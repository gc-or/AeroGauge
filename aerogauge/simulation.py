"""Single rigid-body plant. Forces applied once, at the actual COM."""
from collections import deque
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .vehicle import DATA, R_BS, Vehicle


class Simulation:
    def __init__(self, config):
        self.vehicle = Vehicle()
        self.config = config
        self.dt = float(config["physics_dt_s"])
        self.control_dt = float(config["control_dt_s"])
        if self.dt <= 0 or self.control_dt <= 0:
            raise ValueError("Simulation and control periods must be positive")
        self.substeps = round(self.control_dt / self.dt)
        if self.dt <= 0 or self.substeps < 1 or not np.isclose(
                self.substeps * self.dt, self.control_dt, rtol=0, atol=1e-12):
            raise ValueError("control_dt_s must be a positive integer multiple of physics_dt_s")
        p = config["plant"]
        self.tau = float(p["motor_time_constant_s"])
        delay = float(p["motor_delay_s"])
        self.delay_steps = round(delay / self.dt)
        self.efficiency = np.asarray(p["motor_thrust_scale"], dtype=float)
        shift = np.asarray(p["com_shift_flu_m"], dtype=float)
        if (p["mass_scale"] <= 0 or p["inertia_scale"] <= 0 or self.tau < 0 or delay < 0
                or not np.isclose(self.delay_steps * self.dt, delay, rtol=0, atol=1e-12)
                or self.efficiency.shape != (4,) or np.any(self.efficiency <= 0)
                or shift.shape != (3,)):
            raise ValueError("Invalid plant parameters; delay must be a multiple of physics_dt_s")
        self.allocation = self.vehicle.matrix(shift, p["kappa_m"])
        root = ET.parse(DATA / "wrj/mujoco/drone.xml").getroot()
        root.find("option").set("timestep", str(self.dt))
        root.find("option").set("integrator", "implicitfast")
        inertial = root.find("./worldbody/body/inertial")
        inertial.set("mass", str(self.vehicle.mass * p["mass_scale"]))
        inertial.set("pos", " ".join(map(str, R_BS @ shift)))
        inertia = np.array(self.vehicle.source["inertia_com_kg_m2"]) * p["inertia_scale"]
        inertial.set("fullinertia", " ".join(map(str, [*inertia.diagonal(),
                            inertia[0, 1], inertia[0, 2], inertia[1, 2]])))
        assets = {}
        for mesh in root.findall("./asset/mesh"):
            path = (DATA / "wrj/mujoco" / mesh.get("file")).resolve()
            key = mesh.get("name") + ".obj"
            assets[key] = path.read_bytes()
            mesh.set("file", key)
        world = root.find("worldbody")
        ET.SubElement(world, "geom", name="ground", type="plane", size="20 20 .01",
                      rgba=".25 .28 .32 1", friction=".6 .005 .0001")
        ET.SubElement(world, "light", pos="0 0 4", dir="0 0 -1")
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"), assets)
        self.data = mujoco.MjData(self.model)
        self.body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "drone")
        if self.model.nq != 7 or self.model.nv != 6:
            raise ValueError("Expected a single free rigid-body asset")
        self.com_shift_s = R_BS @ shift
        self.reset(0)

    def reset(self, seed):
        rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        angles = np.deg2rad(rng.uniform(-self.config["initial_tilt_deg"],
                                       self.config["initial_tilt_deg"], 3))
        r_wb = Rotation.from_euler("xyz", angles).as_matrix()
        r_ws = r_wb @ R_BS
        q_xyzw = Rotation.from_matrix(r_ws).as_quat()
        self.data.qpos[3:7] = q_xyzw[[3, 0, 1, 2]]
        half = np.asarray(self.config["initial_position_half_range_m"])
        position = np.array(self.config["initial_position_m"]) + rng.uniform(-half, half)
        self.data.qpos[:3] = position - r_ws @ self.com_shift_s
        # Start ideal motors at nominal trim; lag state is explicitly initialized.
        self.motor = self.vehicle.trim.copy()
        self.queue = deque([self.motor.copy() for _ in range(self.delay_steps)])
        self.clipped = False
        self.had_contact = False
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    def state(self):
        r_ws = self.data.xmat[self.body].reshape(3, 3).copy()
        omega_s = self.data.qvel[3:6].copy()
        velocity = self.data.qvel[:3] + r_ws @ np.cross(omega_s, self.com_shift_s)
        return {"time_s": float(self.data.time), "position": self.data.xipos[self.body].copy(),
                "velocity": velocity, "rotation": r_ws @ R_BS,
                "omega": R_BS @ omega_s}

    def step(self, requested):
        requested = np.asarray(requested, dtype=float)
        if requested.shape != (4,) or not np.isfinite(requested).all():
            raise ValueError("Motor thrust must be four finite values, in new motor order")
        command = np.clip(requested, 0, self.vehicle.max_thrust)
        self.clipped = bool(np.any(command != requested) or np.any(command <= 0)
                            or np.any(command >= self.vehicle.max_thrust))
        for _ in range(self.substeps):
            if self.delay_steps:
                self.queue.append(command.copy())
                delayed = self.queue.popleft()
            else:
                delayed = command
            alpha = 1. if self.tau == 0 else -np.expm1(-self.dt / self.tau)
            self.motor += alpha * (delayed - self.motor)
            wrench = self.allocation @ (self.motor * self.efficiency)
            s = self.state()
            disturbance = self.config["disturbance"]
            force = np.array(disturbance["constant_force_world_N"], dtype=float)
            start, end = disturbance["pulse_interval_s"]
            if start <= self.data.time < end:
                force += disturbance["pulse_force_world_N"]
            # Explicit reduced-order body drag, NOT rotor inflow or ship airwake.
            relative_b = s["rotation"].T @ (s["velocity"] - np.array(disturbance["wind_world_m_s"]))
            drag_b = -.5 * disturbance["air_density_kg_m3"] * np.array(
                disturbance["drag_cd_area_m2"]) * relative_b * np.abs(relative_b)
            force += s["rotation"] @ drag_b
            self.data.xfrc_applied[:] = 0
            self.data.xfrc_applied[self.body, :3] = s["rotation"] @ [0., 0., wrench[0]] + force
            self.data.xfrc_applied[self.body, 3:] = s["rotation"] @ wrench[1:]
            mujoco.mj_step(self.model, self.data)
            # Include contacts at every physics step, not just the control sample.
            if self.data.ncon:
                self.had_contact = True
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                raise FloatingPointError("Nonfinite MuJoCo state")
        return self.state()
