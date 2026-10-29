"""Optional actor-contract checks; no Isaac, CUDA or MuJoCo required."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from aerogauge.policies import TorchScriptHover
from aerogauge.vehicle import Vehicle


@unittest.skipUnless(importlib.util.find_spec("torch"), "Optional PyTorch not installed")
class ExportTests(unittest.TestCase):
    def test_exported_actor_trim_and_reset(self):
        import torch
        module = torch.nn.Linear(16, 4)
        with torch.no_grad():
            module.weight.zero_()
            module.bias.fill_(.2)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"actor.pt"
            torch.jit.trace(module.eval(), torch.zeros(1, 16)).save(str(path))
            contract = json.loads(Path("configs/hover16_contract.example.json").read_text())
            v = Vehicle()
            actor = TorchScriptHover(path, contract, v)
            state = dict(position=np.array([0., 0., 1.]), rotation=np.eye(3),
                         velocity=np.zeros(3), omega=np.zeros(3))
            command = actor.act(state, state["position"], np.zeros(3))
            np.testing.assert_allclose(command, v.trim*1.1, rtol=1e-6)
            np.testing.assert_allclose(actor.previous, .2)
            actor.reset()
            np.testing.assert_array_equal(actor.previous, np.zeros(4))

    def test_other_contract_and_recurrence_rejected(self):
        contract = json.loads(Path("configs/hover16_contract.example.json").read_text())
        contract["interface"] = "landing44"
        with self.assertRaises(ValueError):
            TorchScriptHover("unused.pt", contract, Vehicle())
        contract["interface"] = "wrj_hover16_trim_v1"
        contract["recurrent"] = True
        with self.assertRaises(ValueError):
            TorchScriptHover("unused.pt", contract, Vehicle())


if __name__ == "__main__":
    unittest.main()
