import numpy as np
import argparse
from utils import ARUCO_DICT
import cv2
import sys

aruco_dict_type = ARUCO_DICT["DICT_6X6_100"]
arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)

tag_name = "test-markers.png"

cv2.imwrite(tag_name, tag)
cv2.imshow("ArUCo Tag", tag)
cv2.waitKey(0)
cv2.destroyAllWindows()