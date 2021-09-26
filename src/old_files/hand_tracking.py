import cv2
import mediapipe as mp
import time
import numpy as np
from movement_wrapper import TrajectoryClient
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


class HandTracker:
    def __init__(self):
        print("Initializing Hand-Tracker")
        # Initialize the trajectory client
        self.client = TrajectoryClient()
        print("Moving to Position...")
        # Move to home position
        self.client.send_cartesian_trajectory(
            [[0.5, 0, 0.2, 0, 1, 0, 1]], [3.5], blocking=True)
        # Previous position of the arm
        self.prev_pos_true = np.array([0.5, 0, 0.2])
        self.prev_pos = np.array([0, 0, 0])
        print("Ready!")

    def movement_generator(self, hand_data):
        if hand_data.multi_hand_landmarks == None:
            self.client.send_cartesian_trajectory(
                [[0.6, 0, 0.2, 0, 1, 0, 1]], [3.0])
            self.prev_pos_true = np.array([0.6, 0, 0.2])
            self.prev_pos = np.array([0, 0, 0])
            print("No hands detected")
            return
        # Get the middle point in the hand
        hand_vector = hand_data.multi_hand_landmarks[0].landmark[9]
        hand_vector = np.array([hand_vector.x, 1 - hand_vector.y, 0])
        # Calculate the direction vector of the hand relative to the center of the screen
        dir_vector = hand_vector - np.array([0.5, 0.5, hand_vector[2]])

        # Calculate the distance between the hand and the center of the screen
        # Don't move the arm if the hand is at the right location
        if np.linalg.norm(dir_vector) < 0.075:
            print("Arm is at the right location")
            return

        dir_vector_hat = dir_vector / np.linalg.norm(dir_vector)

        # scaler and hurtz used to calculate the movement of the arm
        scaler = 0.005
        hurtz = 5
        # Refactor the direction to match the arm
        dir_vector_hat = np.array([0, dir_vector_hat[0], dir_vector_hat[1]])
        # Calculate the movement of the arm
        new_movement_rel = scaler * dir_vector_hat + self.prev_pos
        # Calculate the movement of the arm in the true space
        new_movement_true = new_movement_rel + self.prev_pos_true
        # append the quaternion to the movement
        new_movement = np.append(new_movement_true, np.array([0, 1, 0, 1]))
        # Send the movement to the trajectory client

        # Check that the new movement is within range of the maxiumum distance the arm can travel
        if np.linalg.norm(new_movement_true) > 1:
            new_movement = list(new_movement)
            print("Movement: ", str(new_movement) + ",  Magnitude: " +
                  str(np.linalg.norm(new_movement)))
            return

        new_movement = list(new_movement)
        print("Movement: ", str(new_movement) + ",  Magnitude: " +
              str(np.linalg.norm(new_movement_true)))
        if not self.client.is_moving():
            self.client.send_cartesian_trajectory(
                [new_movement], [1 / hurtz])
            # Update the previous position
            self.prev_pos = new_movement_rel
            self.prev_pos_true = new_movement_true

    def start_tracking(self):
        print("Starting Hand-Tracker")
        cap = cv2.VideoCapture(0)
        with mp_hands.Hands(
                min_detection_confidence=0.65,
                min_tracking_confidence=0.65) as hands:
            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.
                    continue

                # Flip the image horizontally for a later selfie-view display, and convert
                # the BGR image to RGB.
                # Rotate the image 90 degrees to match the mediapipe/tensorflow model.
                image = cv2.cvtColor(cv2.rotate(
                    cv2.flip(image, 1), cv2.ROTATE_90_COUNTERCLOCKWISE), cv2.COLOR_BGR2RGB)
                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                results = hands.process(image)

                # Draw the hand annotations on the image.
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style())
                cv2.imshow('MediaPipe Hands', image)

                # Call on movement generator to move the arm based on the hand data
                self.movement_generator(results)

                # Press 'q' to quit.
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            cap.release()


if __name__ == "__main__":
    tracker = HandTracker()
    tracker.start_tracking()

"""
                is_pinched = False

                # If the index finger tip is near the thumb, print a message.
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        # Calculate the distance between the index finger tip and the tip of the thumb.
                        index_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                        thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
                        distance = ((index_finger_tip.x - thumb_tip.x) **
                                    2 + (index_finger_tip.y - thumb_tip.y)**2)**0.5
                        if distance < 0.025:
                            print(
                                'You are holding your thumb close to your index finger.')
                            is_pinched = True
                        else:
                            is_pinched = False

                # If is_pinched is true, send a cartesian trajectory to the robot.
                if is_pinched:
                    client.send_cartesian_trajectory(
                        [[0.35, 0, 0.2, 0, 1, 0, 1]], [3.0], blocking=False)
                else:
                    client.send_cartesian_trajectory(
                        [[0.8, 0, 0.2, 0, 1, 0, 1]], [3.0], blocking=False)
"""
