# written by Branden Kretschmer for Nimble Robotics python coding challenge
# email: brandenkretsch@gmail.com

from typing import Tuple
import cv2 as cv
import numpy as np

import asyncio
import aiortc
from aiortc.contrib.signaling import TcpSocketSignaling, BYE
import av
import ctypes
import argparse
import os

import multiprocessing as multi
from collections import deque

########################################################################################################################
# code for detecting where ball is
########################################################################################################################

class POINT(ctypes.Structure):
    '''
        ctypes structure for use in detect_center_proc. stores an x and y point
        for a circle that has been identified. For use in the val ctypes field
        for communicating between main client process and the detect_center_proc
        process.

        To create, use the following code:
        point = POINT(int, int)
    '''
    _fields_ = [
        ("x", ctypes.c_int),
        ("y", ctypes.c_int),
        ("time_stamp", ctypes.c_int)
    ]

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
    circles = cv.HoughCircles(gray_frame, cv.HOUGH_GRADIENT, 5, 10)
    return circles[0][0] # TODO: check if this indexing is correct

def detect_center_proc(que: multi.Queue, val: multi.Value, cond: multi.Value):
    '''
        TODO: write doc string 
    '''
    try:
        while True:
            if que.qsize() > 0 and cond.value==0:

                # with cond.get_lock():
                frame, timestamp = que.get() # get bgr frame from the que
                # print(timestamp)
                circles = detect_center(frame)
                # print(circles)
                val.x = int(circles[0])
                val.y = int(circles[1])
                val.time_stamp = timestamp
                cond.value = 1
            else:
                # print("empty")
                pass
    except Exception as e:
        return


########################################################################################################################
# code for rtc client
########################################################################################################################

def discard_asyncio_exceptions():
    while True:
        tasks = asyncio.all_tasks()
        for task in tasks:
            pass
        asyncio.wait(0)

class RTCClient():
    '''
        TODO: write doc string
    '''
    def __init__(self, host: str, port: str):
        '''
            TODO: write doc string
            This client only works with a single track coupled with a single datachannel
        '''
        self.host = host
        self.port = port
        self.signal: TcpSocketSignaling = TcpSocketSignaling(self.host, self.port)
        self.pc = aiortc.RTCPeerConnection()
        self.channel: aiortc.RTCDataChannel = None
        self.track: aiortc.MediaStreamTrack = None

    async def connect(self):
        '''
            TODO: write doc
        '''

    async def consume_signal(self) ->bool:
        '''
            waits for a signal response through a tcp connection.

            If the data recieved through the signal is an ICE Candidate or and SDP offer/answer,
            handle that case and return True. If signal was an sdp offer, create an answer and send it

            If not Returns False
        '''
        obj = await self.signal.receive()
        if isinstance(obj, aiortc.RTCSessionDescription):
            await self.pc.setRemoteDescription(obj)
            if obj.type == "offer":
                print("received offer")
                await self.pc.setLocalDescription(await self.pc.createAnswer())
                await self.signal.send(self.pc.localDescription)
            return True
        elif isinstance(obj, aiortc.RTCIceCandidate):
            print("got ICE candidate")
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


class BallVideoRTCClient(RTCClient):
    '''
        TODO: write Doc String
    '''
    def __init__(self, host: str, port: str, display: bool = False):
        '''
            TODO write DOC String
        '''
        super().__init__(host, port)
        self._m = multi.Manager()
        self._proc_que = self._m.Queue()
        self._proc_value = multi.Value(POINT, lock=False)   # don't need a lock because there's only 1 writer and 1 reader. read/writes are synced as well
        self._proc_cond = multi.Value(ctypes.c_int, lock=False) # don't need a lock. 2 states (0,1). 1 writer and 1 reader for each state
        self._proc_value.x = -1
        self._proc_value.y = -1
        self._proc_value.time_stamp = -1
        self._proc_cond.value = 0
        self._frame_proc = multi.Process(target=detect_center_proc, args=(
            self._proc_que,
            self._proc_value,
            self._proc_cond
        ))
        self._frame_proc.start()
        self.display = display

    async def register_on_callbacks(self):
        '''
            Register callbacks when an RTC event happens.
            This client handles two events:
                1) on track event - set track to self.track
                2) on datachannel event - set channel to self.channel
                3) on connectionstatechange event - exit if connection is dropped
        '''
        @self.pc.on("datachannel")
        def on_datachannel(chan: aiortc.RTCDataChannel):
            '''
                When a data channel is connected, set self.channel to this channel
            '''
            print("channel connected")
            self.channel = chan
            chan.send("hi")
        
        @self.pc.on("track")
        def on_track(track: aiortc.MediaStreamTrack):
            '''
                When a track is connected, set self.track to track
            '''
            print("track connected")
            self.track = track

        @self.pc.on("connectionstatechange")
        async def on_connectionstatechange():
            '''
                Log details about connection state. If connection fails, just exit program
            '''
            print(f"connection state is {self.pc.connectionState}")
            if self.pc.connectionState == "failed":
                await self.pc.close()
                exit(-1)
     
    def show_frame(self, ndarr_frame: np.ndarray):
        '''
            print aframe in bgr format if self.display is set to True
        '''
        if self.display:
            cv.imshow('client', ndarr_frame)
            cv.waitKey(1)

    async def _run_track(self) -> bool:
        '''
            Attempts to get the next frame from the track
        '''
        try:
            frame = await self.track.recv()
        except aiortc.mediastreams.MediaStreamError as e:
            print(e)
            print("run track returning false")
            return False
        
        time_stamp = frame.pts
        ndarr_frame = frame.to_ndarray(format = 'bgr24')
        self._proc_que.put((ndarr_frame, time_stamp))
        self.show_frame(ndarr_frame)
        
        if self._proc_cond.value == 1:
            # deal with processed frame aka send it over to server
            # with self.__frame_proc_cond.get_lock():
            # send server the estimated values of the center of circle with the timestamp for it as well
            self.channel.send(f'{self._proc_value.x}\t{self._proc_value.y}\t{self._proc_value.time_stamp}')
            self._proc_cond.value = 0
        else:
            # continue processing stream
            pass
        return True
    
    async def run(self):
        '''
            Method to run server.
            This should be implemented in a subclass
        '''

        await self.register_on_callbacks()
        try:
            while await self.consume_signal():
                print("still consuming")
                if self.track is not None:
                    while await self._run_track():
                        continue
        except Exception as e:
            print("Exception!!")
            print(e)

        await self.shutdown()
    
    async def shutdown(self):
        '''
            Method to shut down server
            This should be implemented in a subclass
        '''
        self._frame_proc.kill()
    
    def __del__(self):
        self._frame_proc.kill()
        #self._proc_value.release()
        #self._proc_cond.release()
        # release queue

async def main():
    '''
        Entry point into client.py. Gets arguments to run script from command line or environment variables.
        Then it builds a client and runs it
    '''
    # get command line args
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
        '-port', '--port',
        action='store',
        help = 'host TCP port number for signaling',
        nargs='?',
        default=None
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

    host = args.host
    port = args.port
    display = args.display
    # if host or port weren't specified in command line, check environment variables
    if host is None:
        host = os.getenv('HOST_TO_CONNECT')
    if port is None:
        port = os.getenv('PORT_TO_CONNECT')
    # if host or port still weren't specified, set default to local host and port 50051
    if host is None:
        host = 'localhost'
    if port is None:
        port = '50051'

    # build client and run it
    client = BallVideoRTCClient(host, port, display=display)
    await client.run()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Key interrupt")

