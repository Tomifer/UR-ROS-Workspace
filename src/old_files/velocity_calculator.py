import sys
import time

from cv2 import magnitude
import rospy
import numpy as np
from movement_wrapper import TrajectoryClient


class VelocityCalculator:
    # This class is used to perfrom all of the neccessary
    # calculation for moveing the robot using veloctity based control

    def __init__(self):
        # Initialize the class
        self.cart_pos = {'x': 0, 'y': 0, 'z': 0}
        self.cart_orient = {'x': 0, 'y': 0, 'z': 0, 'w': 0}
        self.linear_velocity = {'x': 0, 'y': 0, 'z': 0}
        self.angular_velocity = {'x': 0, 'y': 0, 'z': 0}
        self.target_pos = {'x': 0, 'y': 0, 'z': 0}
        self.target_orient = {'x': 0, 'y': 0, 'z': 0, 'w': 0}
        self.velocity_scaler = 0.5
        self.max_linear_velocity = 5
        self.max_distance = 0.9
        self.min_linear_velocity = 0.02
        self.max_velocity_delta = 0.01
        self.client = TrajectoryClient()
        self.update_cartesian_position(initialization=True)

    def __calculate_magnitude(self, vector):
        # This function is used to calculate the magnitude of a vector
        return np.sqrt(vector['x']**2 + vector['y']**2 + vector['z']**2)

    def __calculate_direction_vector(self, initial_pos, final_pos):
        # This function is used to calculate the direction vector from the initial position to the final position
        direction_vector = {'x': final_pos['x'] - initial_pos['x'],
                            'y': final_pos['y'] - initial_pos['y'],
                            'z': final_pos['z'] - initial_pos['z']}
        magnitude = self.__calculate_magnitude(direction_vector)
        direction_vector['x'] = direction_vector['x'] / magnitude
        direction_vector['y'] = direction_vector['y'] / magnitude
        direction_vector['z'] = direction_vector['z'] / magnitude
        return direction_vector, magnitude

    def check_for_completion(self, target_pos, threshold):
        # This function is used to check if the robot has reached the target position
        # It takes in the target position and returns true if the robot has reached the target position
        # Calculate the distance between the current position and the target position
        delta = {'x': target_pos['x'] - self.cart_pos['x'],
                 'y': target_pos['y'] - self.cart_pos['y'],
                 'z': target_pos['z'] - self.cart_pos['z']}
        # Calculate the magnitude of the distance
        magnitude = self.__calculate_magnitude(delta)
        # Check if the magnitude is less than the threshold
        if magnitude < threshold:
            # Set the linear velocity to 0
            self.linear_velocity['x'] = 0
            self.linear_velocity['y'] = 0
            self.linear_velocity['z'] = 0
            self.angular_velocity['x'] = 0
            self.angular_velocity['y'] = 0
            self.angular_velocity['z'] = 0
            self.send_velocity_to_robot()
            return (True, magnitude)
        else:
            return (False, magnitude)

    def calculate_velocity(self, target_pos, ignore_positions=[0, 0, 0]):
        # This function is used to calculate the velocity of the robot
        # It takes in the target position and orientation and returns the velocity
        self.update_cartesian_position()
        # print("Current Position: " + str(self.cart_pos))
        # Check if the target position is within range of robot's limits
        if self.__calculate_magnitude(target_pos) > self.max_distance:
            print("Target position is out of range")
            return

        # Check that we have are close enough to the target position
        if self.check_for_completion(target_pos, 0.001)[0]:
            print("Target Position: " + str(target_pos))
            print("Current Position: " + str(self.cart_pos))
            print("Reached target position, Distance: " +
                  str(self.check_for_completion(target_pos, 0.1)[1]))
            return

        self.target_pos = target_pos

        if ignore_positions[0] == 1:
            target_pos['x'] = self.cart_pos['x']
        if ignore_positions[1] == 1:
            target_pos['y'] = self.cart_pos['y']
        if ignore_positions[2] == 1:
            target_pos['z'] = self.cart_pos['z']

        # Calculate the direction vector from the current position to the target position
        direction_vector, magnitude = self.__calculate_direction_vector(
            self.cart_pos, self.target_pos)

        calculated_velocity = {'x': 0, 'y': 0, 'z': 0}
        # Calculate the linear velocity
        calculated_velocity['x'] = round(direction_vector['x'] *
                                         self.velocity_scaler * (magnitude / self.max_linear_velocity), 5)
        calculated_velocity['y'] = round(direction_vector['y'] *
                                         self.velocity_scaler * (magnitude / self.max_linear_velocity), 5)
        calculated_velocity['z'] = round(direction_vector['z'] *
                                         self.velocity_scaler * (magnitude / self.max_linear_velocity), 5)
        calculated_velocity['x'] += round(direction_vector['x'] *
                                          self.min_linear_velocity, 5)
        calculated_velocity['y'] += round(direction_vector['y'] *
                                          self.min_linear_velocity, 5)
        calculated_velocity['z'] += round(direction_vector['z'] *
                                          self.min_linear_velocity, 5)

        if abs(calculated_velocity['x'] - self.linear_velocity['x']) > self.max_velocity_delta:
            self.linear_velocity['x'] = calculated_velocity['x'] + \
                self.max_velocity_delta * np.sign(calculated_velocity['x'])
        if abs(calculated_velocity['y'] - self.linear_velocity['y']) > self.max_velocity_delta:
            self.linear_velocity['y'] = calculated_velocity['y'] + \
                self.max_velocity_delta * np.sign(calculated_velocity['y'])
        if abs(calculated_velocity['z'] - self.linear_velocity['z']) > self.max_velocity_delta:
            self.linear_velocity['z'] = calculated_velocity['z'] + \
                self.max_velocity_delta * np.sign(calculated_velocity['z'])
        # Calculate the angular velocity
        self.angular_velocity['x'] = 0
        self.angular_velocity['y'] = 0
        self.angular_velocity['z'] = 0
        # Send the velocity to the robot
        self.send_velocity_to_robot()
        # Print the linear velocity
        print("\nLinear Velocity: " + str(self.linear_velocity) +
              ", Mag: " + str(magnitude))
        print("Current Position: " + str(self.cart_pos))
        print("Target Position: " + str(self.target_pos))

        if self.linear_velocity['x'] > 0.004:
            print("==========================================================")

    def get_cartesian_position(self):
        # This function is used to get the current position of the robot as a copy
        return self.cart_pos.copy()

    def send_velocity_to_robot(self):
        # This function is used to send the velocity to the robot
        self.client.twist_controller(
            self.linear_velocity, self.angular_velocity)

    def get_client(self):
        return self.client

    def update_cartesian_position(self, initialization=False):
        # This function is called everytime a new position is published to the /tf topic
        # It updates the current position of the robot
        pos = self.client.get_cartesian_position()

        # Check that the new position is not very different from the old position
        if not initialization:
            direction_vector, magnitude = self.__calculate_direction_vector(
                self.cart_pos, pos[0])
            if magnitude > 0.75:
                return

        self.cart_pos['x'] = round(pos[0]['x'], 5)
        self.cart_pos['y'] = round(pos[0]['y'], 5)
        self.cart_pos['z'] = round(pos[0]['z'], 5)
        self.cart_orient['x'] = round(pos[1]['x'], 5)
        self.cart_orient['y'] = round(pos[1]['y'], 5)
        self.cart_orient['z'] = round(pos[1]['z'], 5)
        self.cart_orient['w'] = round(pos[1]['w'], 5)


if __name__ == "__main__":
    # Initialize the class
    vc = VelocityCalculator()

    home_pos = vc.get_cartesian_position()
    target_pos = vc.get_cartesian_position()
    target_pos['y'] = target_pos['y'] + 0.200
    current_pos = target_pos.copy()
    print("Home Position: " + str(home_pos))
    print("Target Position: " + str(target_pos))
    is_home = True

    while True:
        vc.calculate_velocity(current_pos)
        vc.send_velocity_to_robot()
        time.sleep(0.01)

        if vc.check_for_completion(current_pos, 0.001)[0]:
            current_pos = home_pos.copy()
            is_home = False

        if not is_home and vc.check_for_completion(home_pos, 0.001)[0]:
            exit()
