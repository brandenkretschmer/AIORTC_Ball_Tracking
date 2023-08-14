# written by Branden Kretschmer
# email: brandenkretsch@gmail.com

from typing import Tuple
import cv2 as cv
import numpy as np

import asyncio
import aiortc
from aiortc.contrib.signaling import TcpSocketSignaling
import av
import time

import multiprocessing as multi
import os
from collections import deque

########################################################################################################################
# code for generating video of ball bouncing
########################################################################################################################

class FrameGenerator():
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
        might not be usable
    '''
    frame_gen = FrameGenerator(velocity, radius, resolution)
    while True:
        x_pos, y_pos = frame_gen.get_current_location()
        frame = frame_gen.get_frame()
        frame_gen.increment_position()
        while que.qsize() >= frame_buff_size:
            time.sleep(buffer_sleep)
        que.put((frame, x_pos, y_pos))

class BallVideoStreamTrack(aiortc.VideoStreamTrack):
    '''
        New stream track that will create a video of a ball bouncing across the screen
    '''
    def __init__(self, velocity: int, radius: int, resolution: Tuple[int, int]):
        '''
            TODO: write doc string
        '''
        super().__init__()
        self.velocity = velocity
        self.radius = radius
        self.resolution = resolution
        self.frame_gen = FrameGenerator(velocity, radius, resolution)
        self.ball_locations = deque()

    async def recv(self):
        '''
            TODO: write doc string
        '''
        pts, time_base = await self.next_timestamp()
        x_pos, y_pos = self.frame_gen.get_current_location()
        frame = self.frame_gen.get_frame()
        self.frame_gen.increment_position()
        frame = av.VideoFrame.from_ndarray(frame, format='bgr24')
        frame.pts = pts
        frame.time_base = time_base
        self.ball_locations.append((x_pos, y_pos, pts))
        return frame

########################################################################################################################
# code for rtc server
# Note, the server will be the offerer
# for callbacks, register using the peer connection object with @pc.on("attribute")
########################################################################################################################

class RTC_Server():
    '''
        TODO: write doc string
    '''
    def __init__(self):
        '''
            TODO: write doc string
        '''
        pass

async def run_rtc_server(
        pc: aiortc.RTCPeerConnection, 
        signal: TcpSocketSignaling, 
        velocity: int, 
        radius: int, 
        resolution: Tuple[int, int]):
    '''
        TODO: write doc string
        references https://github.com/aiortc/aiortc/blob/main/examples/videostream-cli/cli.py which is a video stream example
    '''
    stream_track = BallVideoStreamTrack(velocity, radius, resolution)

    # These decorators define callbacks for when an event happens
    # ex: on datachannel event, we will print that a data channel connected
    @pc.on("datachannel")
    def on_datachannel(channel):
        '''
            TODO: write doc string
        '''
        print(f'channel {channel.label} connected')
        
        @channel.on("message")
        def on_message(message):
            '''
                TODO: create docstring and put server side printing here
            '''
            print(f'''
                channel {channel.label} sent message:
                {message}
            ''')

    await signal.connect() # this doesn't do anything, but examples use it so here it is. function is implemented as just a pass in aiortc.
    pc.addTrack(stream_track)
    await pc.setLocalDescription(await pc.createOffer())
    print("sending offer")
    await signal.send(pc.localDescription)

    

async def main():
    '''
        TODO: write doc string
        Entry point of script to start server and connect to clients
    '''
    velocity = 5
    radius = 40
    resolution = (1920, 1080)
    host = 'localhost'
    port = '50001'
    # TODO: move variables above to command line interface or environment variables with defaults to fall back on

    signal = TcpSocketSignaling(host, port)
    pc = aiortc.RTCPeerConnection()
    await run_rtc_server(pc, signal, velocity, radius, resolution)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    # asyncio.run(main())
