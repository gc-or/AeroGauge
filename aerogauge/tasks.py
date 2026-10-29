"""Reference motion; no policy or plant dependencies."""
import numpy as np


def reference(task, t, config):
    position = np.array(config["target_position_m"], dtype=float)
    velocity = np.zeros(3)
    if task == "step":
        if t >= 2.:
            position[0] += .5
    elif task == "circle":
        # Circle starts at the initial target; reference position is continuous.
        w, radius = .5, .4
        position += [radius * (np.cos(w*t)-1), radius*np.sin(w*t), 0.]
        velocity = np.array([-radius*w*np.sin(w*t), radius*w*np.cos(w*t), 0.])
    elif task != "hover":
        raise ValueError(f"Unsupported task: {task}")
    return position, velocity
