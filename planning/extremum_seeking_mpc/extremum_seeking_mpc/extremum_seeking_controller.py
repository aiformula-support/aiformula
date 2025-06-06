import numpy as np

from .util import ControllerParameters, LowPassFilterParameters


class ExtremumSeekingController:
    def __init__(self, params: ControllerParameters, lowpass_params: LowPassFilterParameters, control_period: float):
        # MPC Parameters
        self.seek_gain = params.seek_gain
        self.seek_amp = params.seek_amp
        self.curvature_max = params.curvature_max
        self.curvature_min = params.curvature_min
        self.feedback_gain = params.feedback_gain

        # Setting Sin Wave
        sin_period = params.sin_period
        sin_time = np.arange(0., sin_period, control_period)
        self.sin_array = np.sin(2.*np.pi*(sin_time/sin_period))

        # Setting Seek Points
        # sin divided by 5: fixed value
        # [1., 0.70710678, 0., -0.70710678, -1.]
        self.seek_points = np.hstack(
            [np.flip(self.sin_array[0:3]), self.sin_array[-3: -1]])
        self.seek_points = self.seek_points * self.seek_amp

        # Moving Average
        self.num_moving_average = len(sin_time)

        # LowPass Filter Parameters
        self.lowpass_A = lowpass_params.A
        self.lowpass_B = lowpass_params.B
        self.lowpass_C = lowpass_params.C
        self.state = 0.
        self.state_next = 0.

        # Previous Output
        self.prev_optimal_control_input = 0.

    def apply_risk_moving_average(self, risk_in: np.ndarray) -> float:
        risk_in = np.array(risk_in)
        highpass_indices = [0, 1, 3, 4]
        sin_indices = [2, 1, 5, 6]
        weights = [1.0, 2.0, 2.0, 1.0]

        # Highpass filter
        differences = risk_in[highpass_indices] - risk_in[2]

        # Risk Moving Average
        sin_values = self.sin_array[sin_indices]
        risk_moving_average_out = np.sum(weights * sin_values * differences) / self.num_moving_average

        # LowPass Filter
        self.state_next = self.lowpass_A * self.state + self.lowpass_B * risk_moving_average_out
        risk_moving_average_out = self.lowpass_C * self.state
        self.state = self.state_next

        return risk_moving_average_out

    def optimize_input(self, moving_average: float, backpropagation_value: float) -> float:
        total_input = moving_average + backpropagation_value
        optimal_control_input = self.seek_gain * total_input + self.feedback_gain * self.prev_optimal_control_input
        optimal_control_input = np.clip(optimal_control_input, self.curvature_min, self.curvature_max)
        self.prev_optimal_control_input = optimal_control_input

        return optimal_control_input
