"""Colour-marker segmentation used by both agents (inspection-point markers)."""
import cv2
import numpy as np

HSV = {"orange": ((8, 190, 90), (20, 255, 255)), "yellow": ((24, 170, 90), (34, 255, 255)),
       "cyan": ((84, 170, 80), (98, 255, 255)), "magenta": ((143, 170, 80), (162, 255, 255))}


def marker_mask(rgb, name):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    lo, hi = HSV[name]
    m = cv2.inRange(hsv, np.array(lo), np.array(hi))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return m > 0
