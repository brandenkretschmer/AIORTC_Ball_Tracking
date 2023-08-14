# written by Branden Kretschmer
# email: brandenkretsch@gmail.com

from typing import Tuple
import cv2 as cv
import numpy as np

import asyncio
import aiortc
import time

import multiprocessing as multi
import os
from collections import deque

########################################################################################################################
# code for generating video of ball bouncing
########################################################################################################################

class frame_generator():
    '''
        TODO: write doc string
        Class used to generate video of a ball bouncing across the screen
    '''
    def __init__(self, velocity: int = 5, radius: int = 40, resolution: Tuple[int, int] = (1920, 1080)):
        '''
            TODO: write doc string. Move comments about variables to here
            Initialize variables to start generating frames
        '''
        self.resolution = resolution
        self.velocity = velocity        # pixels per frame that ball will move in both x and y directions
        self.radius = radius            # radius of ball that will be drawn
        self.x_pos = self.radius        # current x position for center of ball
        self.y_pos = self.radius        # current y position for center of ball
        self.x_sense = 1                # direction of x velocity. Positive is to the right.
        self.y_sense = 1                # direction of y velocity. Positive is down.
    
    def get_frame(self):
        '''
            TODO: write doc string
            Generate the next frame
        '''

        # draw frame with ball in current position and add ball position to location queue
        start_time = time.time()
        frame = np.zeros((self.resolution[1], self.resolution[0], 3), dtype='uint8') # image will be in bgr representation
        cv.circle(frame, (self.x_pos, self.y_pos), radius = self.radius, thickness = -1, color = (0, 255, 0)) # TODO: make color customizeable

        return frame
    
    def get_current_location(self):
        '''
            TODO: write doc string
        '''
        return (self.x_pos, self.y_pos)
    
    def increment_position(self):
        '''
            TODO: write doc string
        '''
        # increment position
        self.x_pos += self.velocity * self.x_sense  # TODO: add different velocities for x and y
        self.y_pos += self.velocity * self.y_sense

        # boundary check and move position in other direction if out of bounds
        if self.x_pos < self.radius or self.x_pos > self.resolution[0] - self.radius:
            self.x_sense *= -1
            self.x_pos += 2 * self.velocity * self.x_sense
        if self.y_pos < self.radius or self.y_pos > self.resolution[1] - self.radius:
            self.y_sense *= -1
            self.y_pos += 2 * self.velocity * self.y_sense
        

def generate_video(que: multi.Queue, velocity: int, radius: int, resolution: Tuple[int, int], frame_buff_size: int, buffer_sleep: int):
    '''
        TODO: write doc string
        This function is meant to be run in a seperate process using multiprocessing
    '''
    frame_gen = frame_generator(velocity, radius, resolution)
    while True:
        x_pos, y_pos = frame_gen.get_current_location()
        frame = frame_gen.get_frame()
        frame_gen.increment_position()
        while que.qsize() >= frame_buff_size:
            time.sleep(buffer_sleep)
        que.put((frame, x_pos, y_pos))

class RTC_Server():
    '''
        TODO: write doc string
    '''
    def __init__(self):
        '''
            TODO: write doc string
        '''
        pass

def main():
    '''
        TODO: write doc string
        Entry point of script to start server and connect to clients
    '''
    gen = frame_generator(fps=30, velocity=15)
    video = cv.VideoWriter()
    for i in range(10):
        frame = gen.get_next_frame()
        cv.imshow('ball video', frame)
        if(cv.waitKey(1) == ord('q')):
            break
    print(gen.ball_location)
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
