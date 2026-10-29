"""Baseline controller and an explicit adapter for the archived hover contract."""
import hashlib
from pathlib import Path

import numpy as np


class PositionController:
    def __init__(self, vehicle):
        # Nominal controller stays fixed when the true plant is perturbed.
        self.vehicle = vehicle

    def reset(self):
        pass

    def act(self, state, target, target_velocity):
        v = self.vehicle
        force = v.mass * (np.array([0., 0., 9.81])
                          + 4. * (target-state["position"])
                          + 3. * (target_velocity-state["velocity"]))
        z = force / max(np.linalg.norm(force), 1e-9)
        y = np.cross(z, [1., 0., 0.])
        y /= max(np.linalg.norm(y), 1e-9)
        desired = np.column_stack((np.cross(y, z), y, z))
        r = state["rotation"]
        skew = .5 * (desired.T @ r - r.T @ desired)
        error = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
        omega = state["omega"]
        torque = -np.array([.12, .12, .06])*error - np.array([.035, .035, .025])*omega
        torque += np.cross(omega, v.inertia @ omega)
        thrust = max(0., force @ r[:, 2])
        return v.allocate(np.r_[thrust, torque])


def hover_observation(state, target, previous_action, contract):
    r = state["rotation"]
    obs = np.r_[r.T @ (target-state["position"]) / contract["position_scale_m"],
                r.T @ state["velocity"] / contract["velocity_scale_m_s"],
                state["omega"] / contract["angular_velocity_scale_rad_s"],
                r.T @ np.array([0., 0., -1.]), previous_action]
    if obs.shape != (16,) or not np.isfinite(obs).all():
        raise ValueError("Invalid hover16 observation")
    return obs.astype(np.float32)


class TorchScriptHover:
    def __init__(self, path, contract, vehicle):
        import torch
        if contract["interface"] != "wrj_hover16_trim_v1":
            raise ValueError("Only the archived hover16/trim action contract is implemented")
        if contract["external_normalization"]:
            raise ValueError("Export external normalization inside the TorchScript module first")
        if contract["recurrent"]:
            raise ValueError("Recurrent actors need a dedicated hidden-state adapter")
        self.torch = torch
        self.module = torch.jit.load(str(path), map_location="cpu").eval()
        self.sha256 = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        self.contract = contract
        self.vehicle = vehicle
        self.reset()

    def reset(self):
        self.previous = np.zeros(4, dtype=np.float32)

    def act(self, state, target, target_velocity):
        obs = hover_observation(state, target, self.previous, self.contract)
        with self.torch.inference_mode():
            output = self.module(self.torch.from_numpy(obs).unsqueeze(0))
        action = output.detach().cpu().numpy()
        if action.shape != (1, 4) or not np.isfinite(action).all():
            raise ValueError("Expected a feed-forward actor with finite output shape (1,4)")
        self.previous = np.clip(action[0], -1., 1.)
        return self.vehicle.trim * (1. + self.contract["trim_fraction_scale"] * self.previous)
