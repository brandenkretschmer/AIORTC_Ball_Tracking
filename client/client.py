# written by Branden Kretschmer for Nimble Robotics python coding challenge
# email: brandenkretsch@gmail.com

import cv2 as cv
import numpy as np

import asyncio
import aiortc

########################################################################################################################
# code for detecting where ball is
########################################################################################################################

def detect_center(frame: np.ndarray):
    '''
        TODO: write doc string
    '''
    # convert to grayscale
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # normally the image would be blurred to reduce noise and make the circles more distinct
    # however, this is a bright circle on a black background, so the circle is as crisp as it
    # gets, no need to blur here

    # apply hough circle tranform to get the estimated center of the circle
    num_rows = gray_frame.shape[0]
    circles = cv.HoughCircles(gray_frame, cv.HOUGH_GRADIENT, 3, 10)
    return circles

########################################################################################################################
# code for rtc client
########################################################################################################################

async def main():
    pass

if __name__ == "__main__":
    asyncio.run(main())
