# take target_left and target_right xy (4 numbers)
# add up and /2 == final target xy (repect to cam resolution)
# x > width/2 --> servo goes to right
# x < width/2 --> servo goes to left
# same with y
# in middle then stop 
# hit 0 or 180 then goes back


import cv2
import mediapipe as mp
import logging

## ros2 libarary
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tutorial_interfaces.msg import Control
from tutorial_interfaces.msg import Carspeed

WINDOW_WIDTH = 800

### Config ###
TARGET = 0
# height and width
# 640 x 480
sz1 = 480
sz2 = 640
# E.g.
# for 50cm away bottle image, we had 38.44 pixels between bottle boxes
# for 30cm away bottle image, we had 68.75 pixels between left and right bottles
focal_length = 30 - 38.44 * 50 / 68.75
focal_length = 4.84
# calibrate theta cameras are 5 cms apart
B = 7.1
tantheta = (1 / (40 - focal_length)) * (B / 2) * sz1 / 100

cv_file = cv2.FileStorage()
cv_file.open(r'/home/benny233/ros2_ws/src/ros2_pcnode_test/ros2_pcnode_test/stereoMap.xml', cv2.FileStorage_READ)
stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()

# Ros2 related
msg = Control()
pan = 0
tilt = 0
fire = 0

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
        
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
        
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

# target joint names
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
        
        
# Ros2 node related
class camNode(Node):
    
    def __init__(self):
        super().__init__("cam")
        self.cmd_vel_pub = self.create_publisher(Control, "/pc_side_pub", 10)
        self.controller_subs = self.create_subscription(Joy, "/joy", self.button_callback, 10)
        self.timer = self.create_timer(0.05, self.send_vel_cmd)
        self.get_logger().info("Cam node has been started")
        
    def send_vel_cmd(self):
        global tilt
        global pan
        global fire
        
        msg.fire = fire 

        self.cmd_vel_pub.publish(msg)
    
    def button_callback(self, msg:Joy):
        global fire
        fire = msg.buttons[1]
        string = ("fire: " + str(fire))
        self.get_logger().info(str(string))
        
        
########
## Main#
########     
def main (args=None):
    
    rclpy.init(args=args)
    node = camNode()
    cv2.namedWindow("Right Eye Stream", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Right Eye Stream", WINDOW_WIDTH, int(WINDOW_WIDTH * 0.75))
    cv2.namedWindow("Left Eye Stream", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Left Eye Stream", WINDOW_WIDTH, int(WINDOW_WIDTH * 0.75))

    right_eye_url = "http://192.168.7.58:81/stream"
    left_eye_url = "http://192.168.7.179:81/stream"

    cap_right_eye = cv2.VideoCapture(right_eye_url)
    cap_left_eye = cv2.VideoCapture(left_eye_url)

    # Check if the video captures are successfully opened
    if not cap_right_eye.isOpened():
        logging.error("Right eye stream cannot be opened")
        exit(1)

    if not cap_left_eye.isOpened():
        logging.error("Left eye stream cannot be opened")
        exit(1)

    joint_coords_right = {'Joint 0 (NOSE)': (675, 1044), 'Joint 1 (LEFT_EYE_INNER)': (675, 1032), 'Joint 2 (LEFT_EYE)': (673, 1032), 'Joint 3 (LEFT_EYE_OUTER)': (670, 1031), 'Joint 4 (RIGHT_EYE_INNER)': (687, 1035), 'Joint 5 (RIGHT_EYE)': (690, 1036), 'Joint 6 (RIGHT_EYE_OUTER)': (700, 1039), 'Joint 7 (LEFT_EAR)': (634, 1032), 'Joint 8 (RIGHT_EAR)': (702, 1048), 'Joint 9 (MOUTH_LEFT)': (659, 1049), 'Joint 10 (MOUTH_RIGHT)': (673, 1055), 'Joint 11 (LEFT_SHOULDER)': (505, 1073), 'Joint 12 (RIGHT_SHOULDER)': (752, 1084), 'Joint 13 (LEFT_ELBOW)': (363, 1104), 'Joint 14 (RIGHT_ELBOW)': (894, 1111), 'Joint 15 (LEFT_WRIST)': (268, 1204), 'Joint 16 (RIGHT_WRIST)': (995, 1138), 'Joint 17 (LEFT_PINKY)': (246, 1222), 'Joint 18 (RIGHT_PINKY)': (1031, 1161), 'Joint 19 (LEFT_INDEX)': (258, 1219), 'Joint 20 (RIGHT_INDEX)': (1056, 1164), 'Joint 21 (LEFT_THUMB)': (270, 1213), 'Joint 22 (RIGHT_THUMB)': (1006, 1152), 'Joint 23 (LEFT_HIP)': (443, 1303), 'Joint 24 (RIGHT_HIP)': (571, 1312), 'Joint 25 (LEFT_KNEE)': (568, 1499), 'Joint 26 (RIGHT_KNEE)': (587, 1512), 'Joint 27 (LEFT_ANKLE)': (546, 1702), 'Joint 28 (RIGHT_ANKLE)': (530, 1731), 'Joint 29 (LEFT_HEEL)': (566, 1736), 'Joint 30 (RIGHT_HEEL)': (499, 1757), 'Joint 31 (LEFT_FOOT_INDEX)': (512, 1756), 'Joint 32 (RIGHT_FOOT_INDEX)': (491, 1802)}

    joint_coords_left = {'Joint 0 (NOSE)': (675, 1044), 'Joint 1 (LEFT_EYE_INNER)': (675, 1032), 'Joint 2 (LEFT_EYE)': (673, 1032), 'Joint 3 (LEFT_EYE_OUTER)': (670, 1031), 'Joint 4 (RIGHT_EYE_INNER)': (687, 1035), 'Joint 5 (RIGHT_EYE)': (690, 1036), 'Joint 6 (RIGHT_EYE_OUTER)': (700, 1039), 'Joint 7 (LEFT_EAR)': (634, 1032), 'Joint 8 (RIGHT_EAR)': (702, 1048), 'Joint 9 (MOUTH_LEFT)': (659, 1049), 'Joint 10 (MOUTH_RIGHT)': (673, 1055), 'Joint 11 (LEFT_SHOULDER)': (505, 1073), 'Joint 12 (RIGHT_SHOULDER)': (752, 1084), 'Joint 13 (LEFT_ELBOW)': (363, 1104), 'Joint 14 (RIGHT_ELBOW)': (894, 1111), 'Joint 15 (LEFT_WRIST)': (268, 1204), 'Joint 16 (RIGHT_WRIST)': (995, 1138), 'Joint 17 (LEFT_PINKY)': (246, 1222), 'Joint 18 (RIGHT_PINKY)': (1031, 1161), 'Joint 19 (LEFT_INDEX)': (258, 1219), 'Joint 20 (RIGHT_INDEX)': (1056, 1164), 'Joint 21 (LEFT_THUMB)': (270, 1213), 'Joint 22 (RIGHT_THUMB)': (1006, 1152), 'Joint 23 (LEFT_HIP)': (443, 1303), 'Joint 24 (RIGHT_HIP)': (571, 1312), 'Joint 25 (LEFT_KNEE)': (568, 1499), 'Joint 26 (RIGHT_KNEE)': (587, 1512), 'Joint 27 (LEFT_ANKLE)': (546, 1702), 'Joint 28 (RIGHT_ANKLE)': (530, 1731), 'Joint 29 (LEFT_HEEL)': (566, 1736), 'Joint 30 (RIGHT_HEEL)': (499, 1757), 'Joint 31 (LEFT_FOOT_INDEX)': (512, 1756), 'Joint 32 (RIGHT_FOOT_INDEX)': (491, 1802)}


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
        frame_left_eye =cv2.remap(frame_left_eye, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT,0)
        frame_right_eye =cv2.remap(frame_right_eye, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT,0)
        
        image_rgb_right = cv2.cvtColor(frame_right_eye, cv2.COLOR_BGR2RGB)
        image_rgb_left = cv2.cvtColor(frame_left_eye, cv2.COLOR_BGR2RGB)

        results_right = pose.process(image_rgb_right)
        results_left = pose.process(image_rgb_left)

        # Render skeleton overlay on the right eye stream
        if results_right.pose_landmarks:
            joint_coords_right = get_joint_coordinates(
                results_right.pose_landmarks, sz2, sz1
            )

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
        else:
            annotated_frame_right = frame_right_eye.copy()
            cv2.imshow("Right Eye Stream", annotated_frame_right)

        # Render skeleton overlay on the left eye stream
        if results_left.pose_landmarks:
            joint_coords_left = get_joint_coordinates(results_left.pose_landmarks, sz2, sz1)

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
        else:
            annotated_frame_left = frame_left_eye.copy()
            cv2.imshow("Left Eye Stream", annotated_frame_left)
            # target not detected
            # write a invalid value (x < -180 || x > 180) to indicate ai have no orders for robot
            msg.pan_angle = -999
            msg.tilt_angle = -999

        if results_right.pose_landmarks & results_left.pose_landmarks: 
        # Combine joint coordinates into a single list
            target_left, target_right = extract_joints_coordinates(
                [joint_coords_right, joint_coords_left], TARGET
            )
            
            target_x = target_left[0]
            target_y = target_left[1]
            
            # if the target on the right:
            if target_x > (sz2/2 + 50):
                msg.pan_angle = 1
            # if target on the left:
            elif target_x < (sz2/2+ 50):
                msg.pan_angle = -1
            # if target is in the middle:
            else:
                msg.pan_angle = 0
                
            # if target on the bottom:
            if target_y > sz1/2:
                msg.tilt_angle = 1
            # if target on the top:
            elif target_y < sz1/2:
                msg.tilt_angle = -1
            # if target is in the middle
            else:
                msg.tilt_angle = 0
                
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        
        rclpy.spin_once(node)

    cap_right_eye.release()
    cap_left_eye.release()
    cv2.destroyAllWindows()
    rclpy.shutdown
