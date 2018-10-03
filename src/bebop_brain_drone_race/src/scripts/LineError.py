#!/usr/bin/env python
import cv2
import numpy as np

class LineError():
    def __init__(self, debug = 0):
        self.error = 0
        self.image = None

    def calc_error(self):
        ''' 
        This method calculates the whether the average x value of a red line is to the left,
        right, or centered with the midpoint of an image.

        Args:
            None
        
        Returns:
           None
        '''
        # convert to an HLS image
        hls  = cv2.cvtColor(self.image, cv2.COLOR_RGB2HLS)

        # filter by red
        lower_range = np.array([0,100,100], dtype=np.uint8)
        upper_range = np.array([125,200,200], dtype=np.uint8)
        filtered_image = cv2.inRange(hls, lower_range, upper_range)
        cv2.imwrite("images/mask.jpg", filtered_image)

        # grab the x-values of the edge points
        x_values = np.where(filtered_image == 255)[1]

        # calculate the average of the x values
        average_x = np.average(x_values)

        # grab the midpoint x-value of the image
        midpoint = (self.image.shape[1])/2

        # Return the error
        return average_x - midpoint

    def get_error(self, image):
        '''
        This method returns whether a red line is fixed to the right, left, or centered in an image.

        Args:
            Arg1: An image read in with cv2.imread(...)
        
        Returns:
           -1 if the average x value is to the left of the midpoint of the image.
            0 if the average x value is centered with the midpoint of the image (25px of leeway). 
            1 if the average x value is to the right of the midpoint of the image.
        '''
        self.image = image
        self.calc_error()
        return self.error