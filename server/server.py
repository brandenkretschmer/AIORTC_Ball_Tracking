# written by Branden Kretschmer for Nimble Robotics python coding challenge
# email: brandenkretsch@gmail.com

from typing import Tuple
import cv2 as cv
import numpy as np

import asyncio
import aiortc

from os import environ as env
from collections import deque

class video_generator():
    '''
        Class used to generate video of a ball bouncing across the screen
    '''
    def __init__(self, velocity: int = 5, radius: int = 10, resolution: Tuple(int, int) = (1200, 1200)):
        self.velocity = velocity
        self.radius = radius
        self.x_pos = self.radius // 2 + 1
        self.y_pos = self.radius // 2 + 1 # TODO: might need to change this to -1 for opencv to render a frame correctly
        self.x_sense = 1    # direction of x velocity
        self.y_sense = 1    # direction of y velocity
        self.frame


def main():
    '''Entry point of script to start server and connect to clients'''
    
    pass

if __name__ == "__main__":
    main()
