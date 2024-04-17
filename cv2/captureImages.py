import cv2

right_eye_url = "http://autoaim_right.local:81/stream"
left_eye_url = "http://autoaim_left.local:81/stream"

cap_right_eye = cv2.VideoCapture(right_eye_url)
cap_left_eye = cv2.VideoCapture(left_eye_url)

num = 0

while cap_right_eye.isOpened():

    succesR, imgR = cap_right_eye.read()
    succesL, imgL = cap_left_eye.read()

    k = cv2.waitKey(5)

    if k == 27:  # ESC
        break
    elif k == ord("s"):  # save image key
        cv2.imwrite(
            r"C:\Users\ed700\Downloads\cv2-20240403T074759Z-001\cv2\stereoImage\right\right"
            + str(num)
            + ".png",
            imgR,
        )

        cv2.imwrite(r"C:\Users\ed700\Downloads\cv2-20240403T074759Z-001\cv2\stereoImage\left\left"+ str(num)+ ".png",imgL,)
        print("Image saved")
        num += 1

    cv2.imshow("ImgR", imgR)
    cv2.imshow("ImgL", imgL)

cap_right_eye.release()
cap_left_eye.release()

cv2.destroyAllWindows()
