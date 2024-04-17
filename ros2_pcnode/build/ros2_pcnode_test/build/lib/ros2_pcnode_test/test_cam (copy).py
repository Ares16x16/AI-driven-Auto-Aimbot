import cv2
import mediapipe as mp
import logging
import time
import copy
import math
import numpy as np
import matplotlib.pyplot as plt
import scipy
import scipy.optimize
import torch
import torchvision
import torchvision.transforms.functional as tvtf
from torchvision.models.detection import (
    MaskRCNN_ResNet50_FPN_Weights,
    MaskRCNN_ResNet50_FPN_V2_Weights,
)
from pathlib import Path

WINDOW_WIDTH = 800

### Config ###
TARGET = 0
sz1 = 2160
sz2 = 3840
# E.g.
# for 50cm away bottle image, we had 38.44 pixels between bottle boxes
# for 30cm away bottle image, we had 68.75 pixels between left and right bottles
focal_length = 30 - 38.44 * 50 / 68.75
focal_length = 3.6
# calibrate theta cameras are 5 cms apart
B = 5
tantheta = (1 / (40 - focal_length)) * (B / 2) * sz1 / 100

joint_names = {
    0: "Joint 0 (NOSE)",
    1: "Joint 1 (LEFT_EYE_INNER)",
    2: "Joint 2 (LEFT_EYE)",
    3: "Joint 3 (LEFT_EYE_OUTER)",
    4: "Joint 4 (RIGHT_EYE_INNER)",
    5: "Joint 5 (RIGHT_EYE)",
    6: "Joint 6 (RIGHT_EYE_OUTER)",
    7: "Joint 7 (LEFT_EAR)",
    8: "Joint 8 (RIGHT_EAR)",
    9: "Joint 9 (MOUTH)",
    10: "Joint 10 (MOUTH_LEFT)",
    11: "Joint 11 (MOUTH_RIGHT)",
    12: "Joint 12 (LEFT_SHOULDER)",
    13: "Joint 13 (RIGHT_SHOULDER)",
    14: "Joint 14 (LEFT_ELBOW)",
    15: "Joint 15 (RIGHT_ELBOW)",
    16: "Joint 16 (LEFT_WRIST)",
    17: "Joint 17 (RIGHT_WRIST)",
    18: "Joint 18 (LEFT_PINKY)",
    19: "Joint 19 (RIGHT_PINKY)",
    20: "Joint 20 (LEFT_INDEX)",
    21: "Joint 21 (RIGHT_INDEX)",
    22: "Joint 22 (LEFT_THUMB)",
    23: "Joint 23 (RIGHT_THUMB)",
    24: "Joint 24 (LEFT_HIP)",
    25: "Joint 25 (RIGHT_HIP)",
    26: "Joint 26 (LEFT_KNEE)",
    27: "Joint 27 (RIGHT_KNEE)",
    28: "Joint 28 (LEFT_ANKLE)",
    29: "Joint 29 (RIGHT_ANKLE)",
    30: "Joint 30 (LEFT_HEEL)",
    31: "Joint 31 (RIGHT_HEEL)",
    32: "Joint 32 (LEFT_FOOT_INDEX)",
    33: "Joint 33 (RIGHT_FOOT_INDEX)",
}


def get_joint_coordinates(landmarks, frame_width, frame_height):
    joint_coordinates = {}
    for idx, landmark in enumerate(landmarks.landmark):
        if landmark.visibility < 0 or landmark.presence < 0:
            continue
        joint_name = mp_pose.PoseLandmark(idx).name
        joint_coordinates[f"Joint {idx} ({joint_name})"] = (
            int(landmark.x * frame_width),
            int(landmark.y * frame_height),
        )
    return joint_coordinates


def extract_joints_coordinates(joint_data, target):
    joint_name = joint_names[target]
    joint_left = joint_data[0][joint_name]
    joint_right = joint_data[1][joint_name]
    return joint_left, joint_right


def calculate_distance(horizontal_distance, focal_length, sz1, tantheta):
    try:
        distance = (5 / 2) * sz1 * (1 / tantheta) / horizontal_distance + focal_length
        return distance
    except ZeroDivisionError:
        print("Division error")
    except Exception as e:
        print(f"Error: {e}")


########
# Main #
########
cv2.namedWindow("Right Eye Stream", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Right Eye Stream", WINDOW_WIDTH, int(WINDOW_WIDTH * 0.75))
cv2.namedWindow("Left Eye Stream", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Left Eye Stream", WINDOW_WIDTH, int(WINDOW_WIDTH * 0.75))

right_eye_url = "http://autoaim_right.local:81/stream"
left_eye_url = "http://autoaim_left.local:81/stream"

# testVideo = r"C:\Users\EDWARD\Desktop\FYP\espcam\pexels_videos_2795750 (2160p).mp4"

MODE = "REAL"
# mp.tasks.vision.RunningMode = "LIVE_STREAM"

# Set up the video capture based on the selected mode
if MODE == "TEST_MODE":
    cap_right_eye = cv2.VideoCapture(testVideo)
    cap_left_eye = cv2.VideoCapture(testVideo)
else:
    cap_right_eye = cv2.VideoCapture(right_eye_url)
    cap_left_eye = cv2.VideoCapture(left_eye_url)

# Check if the video captures are successfully opened
if not cap_right_eye.isOpened():
    logging.error("Right eye stream cannot be opened")
    exit(1)

if not cap_left_eye.isOpened():
    logging.error("Left eye stream cannot be opened")
    exit(1)

mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

while True:
    ret_right_eye, frame_right_eye = cap_right_eye.read()
    if not ret_right_eye:
        logging.error("Error reading right eye stream")
        break

    ret_left_eye, frame_left_eye = cap_left_eye.read()
    if not ret_left_eye:
        logging.error("Error reading left eye stream")
        break

    # sz1 = frame_right_eye.shape[0]  # Height
    # sz2 = frame_left_eye.shape[1]  # Width
    # print(sz1, sz2)

    image_rgb_right = cv2.cvtColor(frame_right_eye, cv2.COLOR_BGR2RGB)
    image_rgb_left = cv2.cvtColor(frame_left_eye, cv2.COLOR_BGR2RGB)

    results_right = pose.process(image_rgb_right)
    results_left = pose.process(image_rgb_left)

    # Render skeleton overlay on the right eye stream
    if results_right.pose_landmarks:
        joint_coords_right = get_joint_coordinates(
            results_right.pose_landmarks, sz2, sz1
        )
        # print("Right Eye Joint Coordinates:")
        # print(joint_coords_right)

        mp_drawing = mp.solutions.drawing_utils
        annotated_frame_right = frame_right_eye.copy()
        mp_drawing.draw_landmarks(
            annotated_frame_right,
            results_right.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=4, circle_radius=2),
            mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=4),
        )

        # Add joint labels to the right eye stream window
        for idx, landmark in enumerate(results_right.pose_landmarks.landmark):
            if landmark.visibility < 0 or landmark.presence < 0:
                continue
            joint_label = f"Joint {idx}"
            joint_coords = (int(landmark.x * sz2), int(landmark.y * sz1))
            cv2.putText(
                annotated_frame_right,
                joint_label,
                joint_coords,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow("Right Eye Stream", annotated_frame_right)

    # Render skeleton overlay on the left eye stream
    if results_left.pose_landmarks:
        joint_coords_left = get_joint_coordinates(results_left.pose_landmarks, sz2, sz1)
        # print("Left Eye Joint Coordinates:")
        # print(joint_coords_left)

        mp_drawing = mp.solutions.drawing_utils
        annotated_frame_left = frame_left_eye.copy()
        mp_drawing.draw_landmarks(
            annotated_frame_left,
            results_left.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=4, circle_radius=2),
            mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=4),
        )

        # Add joint labels to the left eye stream window
        for idx, landmark in enumerate(results_left.pose_landmarks.landmark):
            if landmark.visibility < 0 or landmark.presence < 0:
                continue
            joint_label = f"Joint {idx}"
            joint_coords = (int(landmark.x * sz2), int(landmark.y * sz1))
            cv2.putText(
                annotated_frame_left,
                joint_label,
                joint_coords,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        cv2.imshow("Left Eye Stream", annotated_frame_left)

        # Combine joint coordinates into a single list
        joint_coords_combined = [joint_coords_right, joint_coords_left]
        # print(joint_coords_combined)
        target_left, target_right = extract_joints_coordinates(
            joint_coords_combined, TARGET
        )
        # print("Left target:", target_left, "Right target:", target_right)

        horizontal_distance = abs(target_left[0] - target_right[0])
        distance = calculate_distance(horizontal_distance, focal_length, sz1, tantheta)
        print(distance)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap_right_eye.release()
cap_left_eye.release()
cv2.destroyAllWindows()
