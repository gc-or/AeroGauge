from pathlib import Path
import tempfile
import unittest

import numpy as np

from aerogauge.cli import load_config
from aerogauge.evaluation import evaluate
from aerogauge.policies import PositionController, hover_observation
from aerogauge.simulation import Simulation
from aerogauge.vehicle import R_BS, Vehicle


class FlightTests(unittest.TestCase):
    def config(self):
        cfg = load_config()
        cfg["initial_position_half_range_m"] = [0, 0, 0]
        cfg["initial_tilt_deg"] = 0
        return cfg

    def test_asymmetric_trim_cancels_wrench(self):
        v = Vehicle()
        np.testing.assert_allclose(v.allocation @ v.trim, [v.mass*9.81, 0, 0, 0], atol=1e-12)
        self.assertGreater(np.linalg.norm((v.allocation @ np.full(4, v.mass*9.81/4))[1:]), .001)

    def test_trim_is_free_flight_equilibrium(self):
        sim = Simulation(self.config())
        for _ in range(50):
            state = sim.step(sim.vehicle.trim)
        np.testing.assert_allclose(state["position"], [0, 0, 1], atol=1e-8)
        self.assertLess(np.linalg.norm(state["omega"]), 1e-8)
        self.assertFalse(sim.had_contact)

    def test_single_motor_torque_matches_full_inertia(self):
        cfg = self.config()
        cfg["control_dt_s"] = cfg["physics_dt_s"]
        sim = Simulation(cfg)
        thrust = sim.vehicle.trim.copy()
        thrust[0] += .1
        s = sim.step(thrust)
        expected = np.linalg.solve(sim.vehicle.inertia, (sim.allocation @ thrust)[1:])
        np.testing.assert_allclose(s["omega"]/sim.dt, expected, rtol=.01, atol=.002)
        self.assertTrue(np.all(np.sign(expected) == [1, -1, 1]))

    def test_reset_repeats_and_clears_contact(self):
        sim = Simulation(load_config())
        first = sim.reset(42)
        sim.had_contact = True
        sim.step(sim.vehicle.trim)
        second = sim.reset(42)
        np.testing.assert_array_equal(first["position"], second["position"])
        np.testing.assert_array_equal(first["rotation"], second["rotation"])
        self.assertFalse(sim.had_contact)

    def test_mass_bias_does_not_retrim_controller(self):
        cfg = self.config()
        cfg["plant"]["mass_scale"] = 1.1
        sim = Simulation(cfg)
        state = sim.step(sim.vehicle.trim)
        self.assertLess(state["velocity"][2], 0)
        self.assertAlmostEqual(sim.model.body_mass[sim.body], sim.vehicle.mass*1.1)

    def test_com_shift_changes_torque_about_actual_com(self):
        cfg = self.config()
        cfg["plant"]["com_shift_flu_m"] = [.01, 0, 0]
        sim = Simulation(cfg)
        np.testing.assert_allclose(sim.state()["position"], [0, 0, 1], atol=1e-12)
        np.testing.assert_allclose(sim.model.body_ipos[sim.body], R_BS @ [.01, 0, 0])
        self.assertGreater((sim.allocation @ sim.vehicle.trim)[2], 0)

    def test_motor_lag_and_delay(self):
        cfg = self.config()
        cfg["control_dt_s"] = .002
        cfg["plant"]["motor_time_constant_s"] = .02
        cfg["plant"]["motor_delay_s"] = .004
        sim = Simulation(cfg)
        old = sim.motor.copy()
        command = old + .2
        sim.step(command)
        np.testing.assert_array_equal(sim.motor, old)
        sim.step(command)
        np.testing.assert_array_equal(sim.motor, old)
        sim.step(command)
        np.testing.assert_allclose(sim.motor, old+.2*(1-np.exp(-.002/.02)))

    def test_wind_force_sign_and_single_model(self):
        cfg = self.config()
        cfg["disturbance"]["wind_world_m_s"] = [2, 0, 0]
        cfg["disturbance"]["drag_cd_area_m2"] = [.02, .02, .03]
        sim = Simulation(cfg)
        self.assertEqual(sim.model.opt.density, 0)
        self.assertGreater(sim.step(sim.vehicle.trim)["velocity"][0], 0)

    def test_hover16_frame_sign_and_previous_action(self):
        sim = Simulation(self.config())
        s = sim.state()
        c = dict(position_scale_m=.5, velocity_scale_m_s=1, angular_velocity_scale_rad_s=5)
        obs = hover_observation(s, np.array([.5, 0, 1]), np.array([.1, .2, .3, .4]), c)
        np.testing.assert_allclose(obs[:3], [1, 0, 0], atol=1e-7)
        np.testing.assert_allclose(obs[9:12], [0, 0, -1], atol=1e-7)
        np.testing.assert_allclose(obs[12:], [.1, .2, .3, .4])

    def test_invalid_timing_and_nan_actions_rejected(self):
        cfg = self.config()
        cfg["control_dt_s"] = .003
        with self.assertRaises(ValueError):
            Simulation(cfg)
        sim = Simulation(self.config())
        with self.assertRaises(ValueError):
            sim.step([0, 0, float("nan"), 0])

    def test_infeasible_allocation_stays_bounded(self):
        v = Vehicle()
        command = v.allocate(np.array([100., 20., 20., 20.]))
        self.assertTrue(np.all((command >= 0) & (command <= v.max_thrust)))

    def test_success_and_failure_both_saved(self):
        class DeadMotors:
            def reset(self):
                pass

            def act(self, *args):
                return np.zeros(4)

        cfg = self.config()
        cfg["duration_s"] = 3
        sim = Simulation(cfg)
        with tempfile.TemporaryDirectory() as tmp:
            good = evaluate(sim, PositionController(sim.vehicle), "hover", [0, 1], Path(tmp)/"good")
            self.assertEqual(good["success_rate"], 1.)
            bad = evaluate(sim, DeadMotors(), "hover", [0, 1], Path(tmp)/"bad")
            self.assertEqual(bad["success_rate"], 0.)
            self.assertEqual(len(bad["episodes"]), 2)
            self.assertFalse(bad["episodes"][0]["completed"])
            self.assertTrue((Path(tmp)/"bad/trajectory_000.csv").exists())
            with self.assertRaises(FileExistsError):
                evaluate(sim, DeadMotors(), "hover", [0], Path(tmp)/"good")


if __name__ == "__main__":
    unittest.main()
