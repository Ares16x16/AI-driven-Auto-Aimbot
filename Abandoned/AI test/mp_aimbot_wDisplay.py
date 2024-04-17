import cv2
import mss
import numpy as np
import mediapipe as mp
import pyautogui

SCREEN_WIDTH, SCREEN_HEIGHT = pyautogui.size()

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose


def move_mouse_to_coordinate(x, y):
    target_x = int(x * SCREEN_WIDTH)
    target_y = int(y * SCREEN_HEIGHT)
    pyautogui.moveTo(target_x, target_y)


def process_frame(image):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    with mp_pose.Pose(static_image_mode=False) as pose:
        results = pose.process(image_rgb)

        if results.pose_landmarks:
            head_landmark = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]
            head_x, head_y = head_landmark.x, head_landmark.y

            move_mouse_to_coordinate(head_x, head_y)

        mp_drawing.draw_landmarks(
            image,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
            mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2),
        )

    return image


def main():
    cv2.namedWindow("Head Tracking", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Head Tracking", 960, 540)
    with mss.mss() as sct:
        monitor = sct.monitors[1]

        while True:
            frame = np.array(sct.grab(monitor))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            frame = process_frame(frame)

            cv2.imshow("Head Tracking", frame)
            if cv2.waitKey(1) == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
