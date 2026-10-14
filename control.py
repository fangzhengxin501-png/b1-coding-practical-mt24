class PDController:
    """Discrete-time PD controller.

    u[t] = kp * e[t] + kd * (e[t] - e[t-1]),  where e[t] = r[t] - y[t].
    """

    def __init__(self, kp: float = 0.15, kd: float = 0.6, initial_error: float = 0.0):
        """
        Args:
            kp: Proportional gain.
            kd: Derivative gain.
            initial_error: Error assumed at t = -1, i.e. e[-1].
        """
        self.kp = kp
        self.kd = kd
        self._initial_error = initial_error
        self.reset()

    def reset(self) -> None:
        """Restore e[-1] to its initial value; call before each new simulation."""
        self._prev_error = self._initial_error

    def control_input(self, reference: float, output: float) -> float:
        """Return the control action u[t] and store e[t] for the next step.

        Args:
            reference: Target depth r[t].
            output: Measured depth y[t].
        """
        e = reference - output
        u = self.kp * e + self.kd * (e - self._prev_error)
        self._prev_error = e
        return u
