# written by Branden Kretschmer
# email: brandenkretsch@gmail.com

from typing import Tuple
import cv2 as cv
import numpy as np

import asyncio
import aiortc
from aiortc.contrib.signaling import TcpSocketSignaling, BYE
import av
import time
import os

import multiprocessing as multi
from collections import deque
import argparse

########################################################################################################################
# code for generating video of ball bouncing
########################################################################################################################

class FrameGenerator():
    '''
        Class used to generate frames of a ball bouncing across the screen
    '''
    def __init__(self, velocity: int = 5, radius: int = 40, resolution: Tuple[int, int] = (640, 480)):
        '''
            Initialize variables to start generating frames
                resolution = resolution    # resolution of the frames (width, height)
                velocity = velocity        # pixels per frame that ball will move in both x and y directions
                radius = radius            # radius of ball that will be drawn
                x_pos = self.radius        # current x position for center of ball
                y_pos = self.radius        # current y position for center of ball
                x_sense = 1                # direction of x velocity. Positive is to the right.
                y_sense = 1                # direction of y velocity. Positive is down.
        '''
        self.resolution = resolution
        self.velocity = velocity       
        self.radius = radius            
        self.x_pos = self.radius       
        self.y_pos = self.radius        
        self.x_sense = 1               
        self.y_sense = 1              
    
    def get_frame(self):
        '''
            Generate the next frame
        '''

        # draw frame with ball in current position and add ball position to location queue
        start_time = time.time()
        frame = np.zeros((self.resolution[1], self.resolution[0], 3), dtype='uint8') # image will be in bgr representation
        cv.circle(frame, (self.x_pos, self.y_pos), radius = self.radius, thickness = -1, color = (0, 255, 0)) 

        return frame
    
    def get_current_location(self):
        '''
            Return stored center of the ball
        '''
        return (self.x_pos, self.y_pos)
    
    def increment_position(self):
        '''
            Increment the position of the ball by the velocity mulitplied by the sense.
            If ball is out of bounds, reverse the sense and increment in the other direction
        '''
        # increment position
        self.x_pos += self.velocity * self.x_sense
        self.y_pos += self.velocity * self.y_sense

        # boundary check and move position in other direction if out of bounds
        if self.x_pos < self.radius or self.x_pos > self.resolution[0] - self.radius:
            self.x_sense *= -1
            self.x_pos += 2 * self.velocity * self.x_sense
        if self.y_pos < self.radius or self.y_pos > self.resolution[1] - self.radius:
            self.y_sense *= -1
            self.y_pos += 2 * self.velocity * self.y_sense
        

class BallVideoStreamTrack(aiortc.VideoStreamTrack):
    '''
        New stream track that will create a video of a ball bouncing across the screen
    '''
    def __init__(self, velocity: int, radius: int, resolution: Tuple[int, int], ball_location_dict: dict = {}):
        '''
            Initialize variables needed for the stream track. This includes instatiating the grame generator and keeping
            a dictionary for ball location.
        '''
        super().__init__()
        self.velocity = velocity
        self.radius = radius
        self.resolution = resolution
        self.frame_gen = FrameGenerator(velocity, radius, resolution)
        self.ball_location_dict = ball_location_dict # key will be frame timestamp, value will be (x,y) tuple
        self.count = 0

    async def recv(self):
        '''
            Generate and return the next frame in the stream
        '''
        pts, time_base = await self.next_timestamp()
        x_pos, y_pos = self.frame_gen.get_current_location()
        frame = self.frame_gen.get_frame()
        self.frame_gen.increment_position()
        frame = av.VideoFrame.from_ndarray(frame, format='bgr24')
        frame.pts = pts
        frame.time_base = time_base
        self.ball_location_dict[pts] = (x_pos,y_pos)
        self.count += 1
        return frame
    

########################################################################################################################
# code for rtc server
# Note, the server will be the offerer
# for callbacks, register using the peer connection object with @pc.on("attribute")
########################################################################################################################

class RTCServer():
    '''
        Base class of an RTCServer that offers a datachannel and a mediastream to a client.
    '''
    def __init__(self, host: str, port: str, stream_track: aiortc.MediaStreamTrack):
        '''
            Initialize host and port that the server will listen on. Also create RTCPeer connection
            and adds a track and a Datachannel.

            args:
                host: host IP address to listen on ex. Host IP, 127.0.0.1 (localhost), or 0.0.0.0 (listen on all addresses)
                port: TCP port to listen to
                stream_track: content that will be served when connected to

            usage order:
                1) create instance of the class
                2) register callback functions
                3) prepare the offer
                4) send the offer
                5) consume signals
        '''
        # initialize what the RTC Server will send and initialize a data channel
        self.host = host
        self.port = port
        self.signal = TcpSocketSignaling(self.host, self.port)
        self.pc = aiortc.RTCPeerConnection()
        self.channel = self.pc.createDataChannel("RTCchannel")
        self.stream_track = stream_track
        self.pc.addTrack(self.stream_track)

    async def prepare_tcp_rtc_offer(self):
        '''
            creates SDP offer to send to client over tcp
        '''
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(await self.pc.createOffer())

    async def send_offer(self):
        '''
            sends offer to a client that connects with the server
        '''
        await self.signal.send(self.pc.localDescription)
        
    async def consume_signal(self) -> bool:
        '''
            waits for a signal response through a tcp connection.

            If the data recieved through the signal is an ICE Candidate or and SDP offer/answer,
            handle that case and return True

            If not Returns False
        '''
        obj = await self.signal.receive()
        if isinstance(obj, aiortc.RTCSessionDescription):
            await self.pc.setRemoteDescription(obj)
            return True
        elif isinstance(obj, aiortc.RTCIceCandidate):
            await self.pc.addIceCandidate(obj)
            return True
        if obj is BYE:
            print("goodbye")
        return False


    async def register_on_callbacks(self):
        '''
            Register callbacks when an RTC event happens.
            This should be implemented in a subclass
        '''
        raise NotImplementedError
    
    async def run(self):
        '''
            Method to run server.
            This should be implemented in a subclass
        '''
        raise NotImplementedError
    
    async def shutdown(self):
        '''
            Method to shut down server
            This should be implemented in a subclass
        '''
        raise NotImplementedError
        

class BallVideoRTCServer(RTCServer):
    '''
        Class for an RTCServer that serves a video of a ball bouncing across the screen. If the client sends back
        coordinates, the server will calculate the error in the coordinates and attempt to draw the error using
        cv.imshow().
    '''
    def __init__(self, host: str, port: str, velocity: int, radius: int, resolution: Tuple[int, int], display: bool = False):
        self.ball_position = {}
        st = BallVideoStreamTrack(velocity, radius, resolution, self.ball_position)
        super().__init__(host, port, st)
        self.display=display

    def calc_rms_error(self, actual: Tuple[int, int], estimated: Tuple[int, int]):
        '''
            Calculates Root Mean Squared Error for an actual point and an estimated point from the client
        '''
        diff = np.array(estimated) - np.array(actual)
        return float(np.sqrt(np.mean(np.square(diff))))

    def show_error_frame(self, actual: Tuple[int, int], estimated: Tuple[int, int]):
        '''
            Draws original ball frame in green with client's estimate on top with
            a red center and outline.
        '''
        if self.display:
            resolution = self.stream_track.resolution
            radius = self.stream_track.radius
            frame = np.zeros((resolution[1], resolution[0], 3), dtype='uint8')
            cv.circle(frame, actual, radius=radius, color=(0,255,0), thickness=-1)
            cv.circle(frame, actual, radius=1, color=(255,0,0), thickness=5)
            cv.circle(frame, estimated, radius=1, color=(0,0,255), thickness=5)
            cv.circle(frame, estimated, radius=radius, color=(0,0,255), thickness=3)
            cv.imshow("server", frame)
            cv.waitKey(1)


    async def register_on_callbacks(self):
        '''
            Define event callback functions here
        '''
        channel = self.channel
        @channel.on("message")
        def on_message(message):
            '''
                When a message comes over the datachannel, this function gets called
                This function parses the message and then attempts to calculate
                RMS Error and draw the received ball coordinates from the client.
            '''
            values = message.split('\t') # for returned estimated coordinates, server expects a string with format: "{x_pos}\t{y_pos}\t{timestamp}"
            try:
                ball_location_dict = self.stream_track.ball_location_dict
                radius = self.stream_track.radius
                resolution = self.stream_track.resolution
                actual_loc = ball_location_dict[int(values[2])]
                estimated_loc = (int(values[0]), int(values[1]))
                self.show_error_frame(actual_loc, estimated_loc)
                error = self.calc_rms_error(actual_loc, estimated_loc)
                print(f"time stamp {values[2]}:\n\tactual location: "
                      f"{actual_loc}\n\testimated location: {estimated_loc}\n\tRMSE: {error}")
                del ball_location_dict[int(values[2])]
            except Exception as e:
                print(e)
        
        @self.pc.on("connectionstatechange")
        async def on_connectionstatechange():
            '''
                Log details about connection state. if connection fails, just exit program
            '''
            print(f"connection state is {self.pc.connectionState}")
            if self.pc.connectionState == "failed":
                await self.pc.close()
                exit(-1)


    async def run(self):
        '''
            Event loop for running the server
        '''
        await self.register_on_callbacks()
        await self.prepare_tcp_rtc_offer()
        await self.send_offer()
        print("offer received")
        while await self.consume_signal():
            print("signal consumed")
            continue
        await self.shutdown()
        return True
    

    async def shutdown(self):
        '''
            Method to shutdown server
            Attempts to close all connections
        '''
        await self.signal.close()
        try:
            await self.channel.close()
        except TypeError:
            pass
        try:
            await self.pc.close()
        except TypeError:
            pass

async def main():
    '''
        Entry point into server.py. Gets parameters to run script from command line arguments or environment variables.
        Then it builds a BallVideoRTCServer and runs it.
    '''
    # get arguments
    arg_parser = argparse.ArgumentParser(
        prog='client.py',
        description='This program connects to server.py in the same directory. It is a WebRTC client that recieves images of a bouncing ball and estimates its center'
    )

    arg_parser.add_argument(
        '-n', '--host',
        action='store',
        help = 'host Ip address',
        nargs='?',
        default=None
    )

    arg_parser.add_argument(
        '-p', '--port',
        action='store',
        help = 'host TCP port number for signaling',
        nargs='?',
        default=None
    )

    arg_parser.add_argument(
        '-v', '--velocity',
        action='store',
        help = 'Velocity in pixels per frame that the ball moves at in both x and y directions',
        nargs='?',
        default=3,
        type=int
    )

    arg_parser.add_argument(
        '-r', '--radius',
        action='store',
        help = 'radius of the ball in the image being generated by the server',
        nargs='?',
        default=20,
        type=int
    )

    arg_parser.add_argument(
        '-rh', '--resolutionHeight',
        action='store',
        help = 'height of the frames generated in number of pixels',
        nargs='?',
        default=480,
        type=int
    )

    arg_parser.add_argument(
        '-rw', '--resolutionWidth',
        action='store',
        help = 'width of the frames generated in number of pixels',
        nargs='?',
        default=640,
        type=int
    )

    arg_parser.add_argument(
        '-d', '--display', 
        dest='display', 
        action='store_true',
        help = 'Select displaying of frames. Defaults to no if not set'
    )
    arg_parser.add_argument(
        '-nd', '--no-display', 
        dest='display', 
        action='store_false',
        help = 'Select no displaying of frames. Defaults to no if not set'
    )
    arg_parser.set_defaults(display=False)

    args = arg_parser.parse_args()
    
    velocity = args.velocity
    radius = args.radius
    resolution = (args.resolutionWidth, args.resolutionHeight)
    display = args.display
    host = args.host
    port = args.port

    if host is None:
        host = os.getenv('HOST_TO_LISTEN')
    if port is None:
        port = os.getenv('PORT_TO_LISTEN')
    if host is None:
        host = 'localhost'
    if port is None:
        port = '50051'

    # Server can only have one connection at a time, but the while loop spins up a new server if a disconnection happens
    while True:
        server = BallVideoRTCServer(host, port, velocity, radius, resolution, display=display)
        await server.run()
        await server.shutdown()

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
