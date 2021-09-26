
from operator import and_
from visual_servoing_controller import VisualServoingController
import cv2
import mediapipe as mp
import time
import numpy as np
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


class HandTracker:
    def __init__(self):
        print("Initializing Hand-Tracker")
        self.vc = VisualServoingController()
        self.vc.get_client().send_cartesian_trajectory(
            [[0.5, 0, 0.35, 0, 1, 0, 1]], [3.5], blocking=True)
        # Previous position of the arm
        self.prev_pos_true = np.array([0.5, 0, 0.35])
        self.prev_pos = np.array([0, 0, 0])
        self.enabled = False
        self.last_time = time.time()
        self.last_time_2 = time.time()

    def euclidean_distance(self, p1, p2):
        return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5

    def check_for_wolfpack(self, hand_data, num_hands=2):

        if hand_data.multi_hand_landmarks == None:
            return False

        if len(hand_data.multi_hand_landmarks) != num_hands:
            return False

        hands_true = []

        # Check each hand for wolfpack
        for hand in hand_data.multi_hand_landmarks:
            # Get the cordinates of each of the finger tips
            if hand_data.multi_hand_landmarks:
                thumb_tip = hand.landmark[4]
                index_tip = hand.landmark[8]
                middle_tip = hand.landmark[12]
                ring_tip = hand.landmark[16]
                pinky_tip = hand.landmark[20]

                # calculate the distance between thumb and each finger tip
                index_tip_distance = self.euclidean_distance(
                    thumb_tip, index_tip)
                middle_tip_distance = self.euclidean_distance(
                    thumb_tip, middle_tip)
                ring_tip_distance = self.euclidean_distance(
                    thumb_tip, ring_tip)
                pinky_tip_distance = self.euclidean_distance(
                    thumb_tip, pinky_tip)

                # Check that the middle and ring are closer than the index and pinky
                if middle_tip_distance < 0.025 and ring_tip_distance < 0.025:
                    if index_tip_distance > 0.025 and pinky_tip_distance > 0.015:
                        hands_true.append(True)
                        continue
                hands_true.append(False)
        print("Hands: ", hands_true)
        # Check that all hands are true
        return all(hands_true)

    def check_for_pinched_fingers(self, hand_data):
        # find the distance betweeen the index tip and the thumb tip
        thumb_tip = hand_data.multi_hand_landmarks[0].landmark[4]
        index_tip = hand_data.multi_hand_landmarks[0].landmark[8]
        # Convert to numpy array
        thumb_tip = np.array([thumb_tip.x, thumb_tip.y, thumb_tip.z])
        index_tip = np.array([index_tip.x, index_tip.y, index_tip.z])
        # Calculate the distance
        distance = np.linalg.norm(thumb_tip - index_tip)
        # print("Distance between thumb and index: ", distance)
        return distance < 0.025

    def movement_generator(self, hand_data):
        if hand_data.multi_hand_landmarks == None:
            print("No hands detected")
            self.vc.set_target_position(self.prev_pos_true)
            return
            # Get the middle point in the hand
        is_pinched = self.check_for_wolfpack(hand_data)

        if abs(self.last_time_2 - time.time()) < 10:
            self.enabled = True

        if is_pinched and not self.enabled and time.time() - self.last_time > 2:
            self.enabled = True
            self.last_time = time.time()
            self.last_time_2 = time.time()

        hand_vector = hand_data.multi_hand_landmarks[0].landmark[4]
        #print("Z coordinate: ", hand_vector.z)
        hand_vector = np.array([hand_vector.x, 1 - hand_vector.y, 0])
        # Calculate the direction vector of the hand relative to the center of the screen
        dir_vector = hand_vector - np.array([0.5, 0.5, hand_vector[2]])
        # refactor the direction vector to be in the y-z plane
        dir_vector = np.array([0, dir_vector[0], dir_vector[1]])

        if self.enabled:
            self.vc.set_target_position(dir_vector + self.prev_pos_true)
        else:
            self.vc.set_target_position(self.prev_pos_true)

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
                time.sleep(0.01)
            cap.release()


if __name__ == "__main__":
    tracker = HandTracker()
    tracker.start_tracking()
