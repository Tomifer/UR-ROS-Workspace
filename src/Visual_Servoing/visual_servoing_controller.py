import sys
import time

from cv2 import magnitude
import rospy
import numpy as np
from movement_wrapper import TrajectoryClient
from pid_visual_servoing import PIDVisualServoing
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PID_HERTZ = 100.0  # Hz
MAX_VELOCITY = 0.75  # m/s
KP_POS = 2.5
KI_POS = 0.0
KD_POS = 0.0
COMPLETATION_PRECISION = 0.001


class Smoother:
    """
    This class is used to smooth velocity so that no extreme values can occur
    """

    def __init__(self, size):
        """
        Initializes the velocity smoothing class.
        :param size: The size of the array to store the velocity data.
        """
        self.size = size
        self.x_velocity_data = np.zeros(size)
        self.y_velocity_data = np.zeros(size)
        self.z_velocity_data = np.zeros(size)
        self.index = 0

    def update(self, velocity):
        """
        Update the velocity smoothing class.
        :param velocity: The velocity to be smoothed.
        :return: The smoothed velocity.
        """
        # Update the index
        self.index = (self.index + 1) % self.size
        # Update the velocity data
        self.x_velocity_data[self.index] = velocity[0]
        self.y_velocity_data[self.index] = velocity[1]
        self.z_velocity_data[self.index] = velocity[2]
        # Compute the average velocity
        x_velocity = np.mean(self.x_velocity_data)
        y_velocity = np.mean(self.y_velocity_data)
        z_velocity = np.mean(self.z_velocity_data)
        # Return the smoothed velocity
        return np.array([x_velocity, y_velocity, z_velocity])


class VisualServoingController:
    """
    This class is responsible for visual servoing based on the current position of the robot and the desired position. PID control is used to move the robot to the desired position.
    """

    def __init__(self):
        """
        Initializes the visual servoing controller.
        """
        self.trajectory_client = TrajectoryClient()
        # Initialize the trajectory client
        self.trajectory_client.switch_controller("twist_controller")
        # Get the current position from the controller
        current_position = self.trajectory_client.get_current_position()
        self.current_position_linear = np.array(
            [current_position[0]['x'], current_position[0]['y'], current_position[0]['z']])
        self.current_position_angular = np.array(
            [current_position[1]['x'], current_position[1]['y'], current_position[1]['z']])
        # Construct 3 PID controllers for each linear axis
        self.x_servo = PIDVisualServoing(
            "Linear [X] Servo", self.current_position_linear[0], [KP_POS, KI_POS, KD_POS], MAX_VELOCITY, PID_HERTZ)
        self.y_servo = PIDVisualServoing(
            "Linear [Y] Servo", self.current_position_linear[1], [KP_POS, KI_POS, KD_POS], MAX_VELOCITY, PID_HERTZ)
        self.z_servo = PIDVisualServoing(
            "Linear [Z] Servo", self.current_position_linear[2], [KP_POS, KI_POS, KD_POS], MAX_VELOCITY, PID_HERTZ)
        # Create a dictionary to store the PID controllers
        self.pid_controllers = {
            'x': self.x_servo,
            'y': self.y_servo,
            'z': self.z_servo
        }
        self.target_position = self.current_position_linear
        # Create smoothing object for the position
        self.position_smoother = Smoother(1)
        self.velocity_smoother = Smoother(4)

    def get_client(self):
        """
        Get the trajectory client of the visual servoing controller.
        :return: The trajectory client of the visual servoing controller.
        """
        return self.trajectory_client

    def target_reached(self):
        """
        Check if the target position has been reached.
        :return: True if the target position has been reached, False otherwise.
        """
        return np.linalg.norm(self.current_position_linear - self.target_position) < COMPLETATION_PRECISION

    def update(self):
        """
        Update the visual servoing controller.
        """
        # Get the current position from the controller
        current_position = self.trajectory_client.get_current_position()
        # Convert the current position to a numpy array
        current_position_linear = np.array(
            [current_position[0]['x'], current_position[0]['y'], current_position[0]['z']])
        current_position_angular = np.array(
            [current_position[1]['x'], current_position[1]['y'], current_position[1]['z']])

        # Print the current position and the target position and the difference
        # print("\nCurrent Position: ", current_position_linear)
        # print("Target Position: ", self.target_position)
        # print("Difference: ", current_position_linear - self.target_position)

        # Filter the updated position if large change in position occurs
        if np.linalg.norm(self.current_position_linear - current_position_linear) < 0.2:
            self.current_position_linear = current_position_linear
            self.current_position_angular = current_position_angular

        # Check if the target position has been reached
        if np.linalg.norm(self.current_position_linear - self.target_position) < COMPLETATION_PRECISION:
            self.current_position_linear = self.target_position
            self.current_position_angular = np.array([0.0, 0.0, 0.0])
            self.trajectory_client.send_velocity(np.array([0.0, 0.0, 0.0]))
            return

        # Update the PID controllers with the new current position
        x_velocity = self.x_servo.update(self.current_position_linear[0])
        y_velocity = self.y_servo.update(self.current_position_linear[1])
        z_velocity = self.z_servo.update(self.current_position_linear[2])
        # Construct the desired velocity
        desired_velocity = np.array([x_velocity, y_velocity, z_velocity])
        # Smooth the desired velocity
        desired_velocity = self.velocity_smoother.update(desired_velocity)
        # print("Desired Velocity: {}".format(desired_velocity))
        if desired_velocity[0] > 0.4:
            print(self.current_position_linear)
            print(self.target_position)
        # Send the desired velocity to the controller
        self.trajectory_client.send_velocity(desired_velocity)

    def set_target_position(self, position):
        """
        Set the target position of the visual servoing controller.
        :param position: The target position of the visual servoing controller.
        """
        # Check that the target position is within reach of the robot
        if np.linalg.norm(np.array(position[0:3])) > 1.0:
            print("Target Position is out of reach!")
            print("Target Position: {}".format(position))
            print("Current Position: {}".format(self.current_position_linear))
            return

        self.target_position = np.array(position[0:3])
        # Update the position smoother
        print("Target Position: {}".format(self.target_position))
        print("Current Position: {}".format(self.current_position_linear))
        self.target_position = self.position_smoother.update(
            self.target_position)
        self.update()
        # Update the PID controllers with the new target position
        self.x_servo.set_target_position(self.target_position[0])
        self.y_servo.set_target_position(self.target_position[1])
        self.z_servo.set_target_position(self.target_position[2])

    def get_current_position(self):
        """
        Get the current position of the visual servoing controller.
        :return: The current position of the visual servoing controller.
        """
        return self.current_position_linear.copy()


if __name__ == "__main__":
    vsc = VisualServoingController()
    current_position = vsc.get_current_position()
    new_position = current_position + np.array([0.0, 0.2, 0.0])
    vsc.set_target_position(new_position)
    y_data = [[], []]
    count = 0
    while not rospy.is_shutdown():
        time.sleep(0.01)
        y_data[0].append(vsc.get_current_position()[1])
        y_data[1].append(new_position[1])

        # plot the y data as an animation using canvas draw and flush
        plt.clf()
        plt.plot(y_data[0], label='y_data')
        plt.plot(y_data[1], label='y_target')
        plt.legend()
        plt.pause(0.001)
        plt.draw()

        # time.sleep(1 / PID_HERTZ)
        vsc.update()

        if vsc.target_reached():
            new_position = current_position
            count += 1
            vsc.set_target_position(current_position)
            if count > 5:
                time.sleep(4)
                break
