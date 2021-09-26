import sys
import time

from cv2 import magnitude
import rospy
import numpy as np
from movement_wrapper import TrajectoryClient

# Import the PID controller library
from simple_pid import PID


class PIDVisualServoing:
    """
    This class is intended to be used for visual servoing using a PID controller for smooth trajectory generation.
    """

    def __init__(self, axis_name, initial_position, pid_params_pos, max_velocity, frequency):
        """
        Initialize the PID controller.

        :param axis_name: The axis name to be used for the PID controller.
        :param initial_position: The initial position of the axis.
        :param pid_params_pos: The PID parameters.
        :param pid_params_vel: The PID parameters.
        :param max_velocity: The maximum velocity of the axis.
        :param frequency: The frequency of the PID controller.
        """
        print("\n===== Initializing PID controller for axis {}... =====".format(axis_name))
        print("Initial position: {} Meters".format(initial_position))
        print("PID parameters position: {}".format(pid_params_pos))
        print("Max velocity: {} M/S".format(max_velocity))
        print("Frequency: {} Hz".format(frequency))
        # Initialize the PID controller
        self.pid_pos = PID(pid_params_pos[0], pid_params_pos[1], pid_params_pos[2],
                           setpoint=initial_position, output_limits=(-max_velocity, max_velocity))
        self.pid_pos.sample_time = 1.0 / frequency
        # Set the axis name
        self.axis_name = axis_name
        self.velocity = 0.0
        print("=====   PID controller for axis {} initialized   =====".format(axis_name))

    def set_target_position(self, target_position):
        """
        Set the target position.

        :param target_position: The target position.
        """
        self.pid_pos.setpoint = target_position

    def update(self, current_position):
        """
        Update the PID controller.

        :param current_position: The current position.
        :return: The PID output.
        """
        # Return the PID output
        return self.pid_pos(current_position)


if __name__ == "__main__":
    x_axis = PIDVisualServoing("X-Axis", 0.0, (1.0, 0.2, 0.4), 1.0, 100.0)
