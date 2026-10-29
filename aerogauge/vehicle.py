"""Nominal vehicle model and bounded thrust allocation in control FLU."""
import json
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear

DATA = Path(__file__).parent / "data"
R_BS = np.diag([-1., -1., 1.])
MOTOR_ORDER = ("motor_3", "motor_1", "motor_4", "motor_2")
REACTION_SIGNS = np.array([1., -1., -1., 1.])


class Vehicle:
    def __init__(self):
        self.source = json.loads((DATA / "wrj/config/vehicle.json").read_text(encoding="utf-8"))
        self.profile = json.loads((DATA / "motor_profile.json").read_text(encoding="utf-8"))
        self.mass = self.source["mass_kg"]
        self.inertia = R_BS @ np.array(self.source["inertia_com_kg_m2"]) @ R_BS.T
        motors = {m["name"]: m for m in self.source["motors"]}
        self.positions = np.array([motors[n]["position_m"] for n in MOTOR_ORDER]) @ R_BS.T
        self.kappa = self.profile["yaw_model"]["kappa_m"]
        self.max_thrust = self.profile["thrust_model"]["max_thrust_N"]
        self.allocation = self.matrix()
        self.trim = np.linalg.solve(self.allocation, [self.mass * 9.81, 0., 0., 0.])

    def matrix(self, com_shift=(0., 0., 0.), kappa=None):
        r = self.positions - np.asarray(com_shift)
        return np.vstack((np.ones(4), r[:, 1], -r[:, 0],
                          REACTION_SIGNS * (self.kappa if kappa is None else kappa)))

    def allocate(self, wrench):
        thrust = np.linalg.solve(self.allocation, wrench)
        if np.all((thrust >= 0) & (thrust <= self.max_thrust)):
            return thrust
        weights = np.diag([1., 10., 10., 10.])
        return lsq_linear(weights @ self.allocation, weights @ wrench,
                          bounds=(0., self.max_thrust)).x
