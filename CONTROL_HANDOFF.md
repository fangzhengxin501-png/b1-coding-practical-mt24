# Control module notes

`control.py` (repo root) contains the PD controller:

```text
u[t] = Kp * e[t] + Kd * (e[t] - e[t-1]),   e[t] = r[t] - y[t],   e[-1] = 0
```

```python
from control import PDController

controller = PDController()                      # kp=0.15, kd=0.6 by default
u = controller.control_input(reference, output)  # returns the control action
controller.reset()                               # e[-1] back to 0
```

The controller stores `e[t-1]` itself and computes the error from the reference and output. `ClosedLoop` only needs to pass the signals in.

## What is needed from software side

In `ClosedLoop.simulate` (`uuv_mission/dynamic.py`):

```python
self.plant.reset_state()
self.controller.reset()        # fresh e[-1] for each run

for t in range(T):
    positions[t] = self.plant.get_position()
    observation_t = self.plant.get_depth()
    actions[t] = self.controller.control_input(mission.reference[t], observation_t)
    self.plant.transition(actions[t], disturbances[t])
```

This interface is just `control_input(reference, output)` plus `reset()`. 
