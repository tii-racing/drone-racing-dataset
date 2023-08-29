from glob import glob
import os
import signal
import sys

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image  

IMAGE_PATH = "data/images/"
LABEL_PATH = "data/labels/"

def signal_handler(_, __):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

images = sorted(glob(IMAGE_PATH + "*"))
colors = ["r", "g", "b", "y"]
keypoint_colors = ["yellow", "lime", "cyan", "magenta"]

for image in images:
    with open(os.path.join(LABEL_PATH, os.path.basename(image).split(".")[0] + ".txt"), "r") as txt:  # read Yolo label in txt format
        img = Image.open(image)  # read the target image
        plt.imshow(img)

        for index, label in enumerate(txt.readlines()):
            values = label.split(" ")

            bb = np.array(values[1:5], dtype=np.float32)
            kps = np.array(values[5:], dtype=np.float32).reshape((4, 3))

            # yolo uses x, y, w, h normalized to the image size. We need to convert them to pixel values.
            w = int(bb[2] * img.size[0])
            h = int(bb[3] * img.size[1])
            x = int(bb[0] * img.size[0] - w/2)
            y = int(bb[1] * img.size[1] - h/2)
            rect = patches.Rectangle((x, y), w, h, linewidth=3, edgecolor=colors[index], facecolor='none')
            plt.gca().add_patch(rect)

            for kp_index, kp in enumerate(kps):
                if kp[0] == 0 and kp[1] == 0:
                    continue
                x = int(kp[0] * img.size[0])
                y = int(kp[1] * img.size[1])
                # non-visible corners are draw in white.
                color = keypoint_colors[kp_index] if kp[2] == 2 else "w"
                plt.scatter(x, y, marker="o", color=color, s=18)

        plt.axis('off')  # Turn off axis ticks and labels
        plt.show()
