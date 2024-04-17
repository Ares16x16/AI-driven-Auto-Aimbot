import copy
import math
import numpy as np
import cv2
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

import stereo_image_utils
from stereo_image_utils import (
    get_detections,
    get_cost,
    draw_detections,
    annotate_class2,
)
from stereo_image_utils import (
    get_horiz_dist_corner_tl,
    get_horiz_dist_corner_br,
    get_dist_to_centre_tl,
    get_dist_to_centre_br,
)

### PARAM ###
testVideo = r"C:\Users\EDWARD\Desktop\FYP\espcam\pexels_videos_2795750 (2160p).mp4"

# pre-calibrated in stereo_image_v6 notebook (TODO)
fl = 2.043636363636363  # Focal length
tantheta = 0.7648732789907391

### AI config ###
weights = MaskRCNN_ResNet50_FPN_V2_Weights.DEFAULT
COLOURS = [
    tuple(int(colour_hex.strip("#")[i : i + 2], 16) for i in (0, 2, 4))
    for colour_hex in plt.rcParams["axes.prop_cycle"].by_key()["color"]
]
model = torchvision.models.detection.maskrcnn_resnet50_fpn_v2(weights=weights)
_ = model.eval()


if __name__ == "__main__":
    cap = cv2.VideoCapture(testVideo)

    while True:
        ret, frame = cap.read()

        if ret:
            # do object detection
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            imgs = [img, img]

            det, lbls, scores, masks = get_detections(model, imgs)

            sz1 = frame.shape[1]
            centre = sz1 / 2

            cost = get_cost(det, lbls=lbls, sz1=centre)
            tracks = scipy.optimize.linear_sum_assignment(cost)

            dists_tl = get_horiz_dist_corner_tl(det)
            dists_br = get_horiz_dist_corner_br(det)

            final_dists = []
            dctl = get_dist_to_centre_tl(det[0], cntr=centre)
            dcbr = get_dist_to_centre_br(det[0], cntr=centre)

            for i, j in zip(*tracks):
                if dctl[i] < dcbr[i]:
                    final_dists.append(
                        (
                            dists_tl[i][j],
                            np.array(weights.meta["categories"])[lbls[0]][i],
                        )
                    )

                else:
                    final_dists.append(
                        (
                            dists_br[i][j],
                            np.array(weights.meta["categories"])[lbls[0]][i],
                        )
                    )

            # final distances as list
            fd = []

            for dist, label in final_dists:
                fd.append(dist)

            # annotate distance on the frame
            frame = annotate_class2(frame, det[0], fd, sz1)

            # display the frame
            cv2.imshow("Object Detection", frame)

            # exit if Esc key is pressed
            if cv2.waitKey(1) == 27:
                break
        else:
            break

    # release the video capture object and close the window
    cap.release()
    cv2.destroyAllWindows()
