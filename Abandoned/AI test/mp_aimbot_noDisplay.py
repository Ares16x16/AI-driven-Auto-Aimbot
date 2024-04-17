import cv2
import mss
import numpy as np
import mediapipe as mp
import pyautogui

mp_holistic = mp.solutions.holistic


def move_mouse_to_coordinate(x, y):
    screen_width, screen_height = pyautogui.size()
    target_x = int(x * screen_width)
    target_y = int(y * screen_height)

    pyautogui.moveTo(target_x, target_y)


def process_frame(frame):
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    with mp_holistic.Holistic(
        static_image_mode=False, min_detection_confidence=0.5
    ) as holistic:
        results = holistic.process(rgb_frame)

        if results.pose_landmarks:
            head_landmark = results.pose_landmarks.landmark[
                mp_holistic.PoseLandmark.NOSE
            ]
            head_x, head_y = head_landmark.x, head_landmark.y

            move_mouse_to_coordinate(head_x, head_y)

        return results.pose_landmarks


def main():
    with mss.mss() as sct:
        monitor = sct.monitors[1]

        while True:
            frame = np.array(sct.grab(monitor))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            landmarks = process_frame(frame)

            if landmarks:
                print("Skeleton Detected!")
            else:
                print("No Skeleton Detected.")


if __name__ == "__main__":
    main()
