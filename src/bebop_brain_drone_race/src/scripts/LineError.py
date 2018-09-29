#!/usr/bin/env python
import cv2
import numpy as np
from math import sqrt, atan

class LineError():
    def __init__(self, debug = 0):
        self.delta = 0
        self.image = None
        self.debug = debug

    def CalcError(self):
        # apply a Gaussian blur and convert to an HLS image
        blur = cv2.blur(self.image,(5,5))
        hls  = cv2.cvtColor(blur, cv2.COLOR_RGB2HLS)

        # apply a color filter
        lower_range = np.array([100, 100, 65], dtype=np.uint8)
        upper_range = np.array([255, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hls, lower_range, upper_range)
        masked_image = cv2.bitwise_and(self.image, self.image, mask=mask)

        # apply Canny Edge Detection
        edges = cv2.Canny(masked_image, 50, 150, apertureSize = 3)

        # apply Hough Lines algorithm
        lines = cv2.HoughLines(edges, 1, np.pi/180, 100)

        # definitions
        h, w = self.image.shape[:2]
        x1s, y1s, x2s, y2s = [], [], [], []

        # parse results for the two most prominent lines
        if lines is not None:
            for line in lines[:2]:
                a = np.cos(line[0][1])
                b = np.sin(line[0][1])
                x0 = a*line[0][0]
                y0 = b*line[0][0]

                x1 = int(x0 + 100000*(-b))
                y1 = int(y0 + 100000*(a))
                x2 = int(x0 - 100000*(-b))
                y2 = int(y0 - 100000*(a))

                if(self.debug == 1):
                    cv2.line(self.image,(x1,y1),(x2,y2),(0,255,0),5)

                x1s.append(x1)
                y1s.append(y1)
                x2s.append(x2)
                y2s.append(y2)

            # calculate the value of delta
            x1 = sum(x1s)//len(x1s)
            x2 = sum(x2s)//len(x2s)
            y1 = sum(y1s)//len(y1s)
            y2 = sum(y2s)//len(y2s)


            if(x1 > w/2):
                self.delta = -1
            elif(x1 < w/2):
                self.delta = 1
            else:
                self.delta = 0

    def GetError(self, image):
        self.image = image
        self.CalcError()
        return self.delta

    def GetImage(self):
        return self.image
