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

def detect_center(frame: np.ndarray, dp: float = 6, minDist: float = 5) -> list[int, int, int]:
    '''
        This function estimates the center of a circle in an image using the Hough Transformation
        input:
            frame = ndarray in BGR24 format representing picture
            dp = accumulator matrix scale factor
            minDist = minimum distance between estimated circle centers
        returns:
            list[x,y, r] = (x_coord, y_coord, radius)
        Note: this function needs to be tuned
    '''
    # convert to grayscale
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # normally the image would be blurred to reduce noise and make the circles more distinct
    # however, this is a bright circle on a black background, so the circle is as crisp as it
    # gets, no need to blur here

    # apply hough circle tranform to get the estimated center of the circle
    num_rows = gray_frame.shape[0]
    circles = cv.HoughCircles(gray_frame, cv.HOUGH_GRADIENT, dp=dp, minDist=minDist)
    if circles is not None:
        return circles[0][0]
    return None


def update_center_values(que: multi.Queue, val: multi.Value, cond: multi.Value, dp: float = 6, minDist: float = 5):
    '''
        This should only be called in detect_center_proc. This was pulled out of the process loop
        in order to write a unit test for it.
        updates values when appropriate
    '''
    if que.qsize() > 0 and cond.value==0:
        with cond.get_lock():
            frame, timestamp = que.get() # get bgr frame from the que
            circles = detect_center(frame, dp, minDist)
            if circles is None:
                # if algorithm cant find center of circle, use last estimated position
                print(f"skipped frame at timestamp {timestamp}")
                val.time_stamp = timestamp
                cond.value = 1
            else:
                # print(circles)
                val.x = int(circles[0])
                val.y = int(circles[1])
                val.time_stamp = timestamp
                cond.value = 1

def detect_center_proc(que: multi.Queue, val: multi.Value, cond: multi.Value, dp: float = 6, minDist: float = 5):
    '''
        Coroutine to run detect_center function within a process.
        Updates val with estimated center of circle in the frame that was popped from que during the same iteration.
        Waits until main thread has dealt with val to pop another frome from que and start analyzing it 

        inputs:
            que = used to pass frames and timestamps to process
            val = used to pass image center coordinates and corresponding timestamp back to main thread
            cond = condition for whether or not the main thread has dealt with the value in val. 0 nor no it hasn't
    '''
    try:
        while True:
            update_center_values(que, val, cond, dp, minDist)
    except Exception as e:
        return


########################################################################################################################
# code for rtc client
########################################################################################################################

class RTCClient():
    '''
        A client that consumes RTC content. Client consumes one MediaStreamTrack and one Datachannel 
    '''
    def __init__(self, host: str, port: str):
        '''
            Initialize parameters for Client

            inputs:
                host = host IP address to connect to
                port = host port number to connect to
        '''
        self.host = host
        self.port = port
        self.signal: TcpSocketSignaling = TcpSocketSignaling(self.host, self.port)
        self.pc = aiortc.RTCPeerConnection()
        self.channel: aiortc.RTCDataChannel = None
        self.track: aiortc.MediaStreamTrack = None


    async def consume_signal(self) ->bool:
        '''
            waits for a signal response through a tcp connection.

            If the data recieved through the signal is an ICE Candidate or and SDP offer/answer,
            handle that case and return True. If signal was an sdp offer, create an answer and send it

            If not Returns False
        '''
        obj = None
        while True:
            # wait for a connection to the server. If the server is not running, 
            # receive will result in an exception, so keep eating the
            try:
                obj = await self.signal.receive()
                break
            except (FileNotFoundError, OSError):
                pass

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
        An RTCClient meant to consume a BallVideoStreamTrack (defined in server), process the frame, and
        eventually send back an estimated center of the ball from the consumed frame.
    '''
    def __init__(self, host: str, port: str, display: bool = False, dp: float= 6, minDist: float = 5):
        '''
            Initializes values needed and also starts _frame_proc process for analyzing frames.
           _proc_value.x represents the extimated x coordinate of the circle from the last analyzed frame
           _proc_value.y is the same as x but for the y coordinate
           _proc_que is used to send frames _frame_proc 
        '''
        super().__init__(host, port)
        self._m = multi.Manager()
        self._proc_que = self._m.Queue()
        self._proc_value = multi.Value(POINT, lock=False)   # don't need a lock because there's only 1 writer and 1 reader. read/writes are synced as well
        self._proc_cond = multi.Value(ctypes.c_int, lock=True)
        self._proc_value.x = -1
        self._proc_value.y = -1
        self._proc_value.time_stamp = -1
        self._proc_cond.value = 0
        self._frame_proc = multi.Process(target=detect_center_proc, args=(
            self._proc_que,
            self._proc_value,
            self._proc_cond,
            dp,
            minDist
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
            print aframe in bgr format if self.display is set to True.
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
        ndarr_frame = frame.to_ndarray()
        ndarr_frame = cv.cvtColor(ndarr_frame, cv.COLOR_YUV2BGR_I420)
        self._proc_que.put((ndarr_frame, time_stamp))
        self.show_frame(ndarr_frame)
        
        with self._proc_cond.get_lock():
            if self._proc_cond.value == 1:
                # deal with processed frame aka send it over to server
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
            
            Registers callbacks, then calls consume signal. Once WebRTC has done its thing and there's a peer 
            to peer connection, the track being served starts to get consumed and keeps getting consumed.
            When consumption stops due to connection ending, call shutdown and end the client process
        '''

        await self.register_on_callbacks()
        while await self.consume_signal():
            print("still consuming")
            if self.track is not None:
                while await self._run_track():
                    pass
        await self.shutdown()
    

    async def shutdown(self):
        '''
            Method to shut down client. Makes sure frame analysis process is killed
        '''
        self._frame_proc.kill()
    

    def __del__(self):
        '''
            make sure frame analysis process gets killed when destructor is called
        '''
        self._frame_proc.kill()


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

    arg_parser.add_argument(
        "--dp",
        action='store',
        help = 'Accumulator Matrix scale for detecting center of circle. Should leave at default value if using default resolution/radius/etc',
        nargs='?',
        default=6,
        type=float
    )

    arg_parser.add_argument(
        "--minDist",
        action='store',
        help = 'Minimum distance between centers for detecting center of circle. Should leave at default value if using default resolution/radius/etc',
        nargs='?',
        default=8,
        type=float
    )
    arg_parser.set_defaults(display=False)

    args = arg_parser.parse_args()

    dp = args.dp
    print(dp)
    minDist = args.minDist
    host = args.host
    port = args.port
    display = args.display
    # if host or port weren't specified in command line, check environment variables
    # The following block of code gets a service IP and Port to contact the server when running in Kubernetes
    if host is None or port is None:
        try:
            service_name = os.getenv("SERVER_SERVICE_NAME")
            host = os.getenv(f"{service_name}_SERVICE_HOST")
            port = os.getenv(f"{service_name}_SERVICE_PORT")
        except Exception as e:
            print(e)
    
    # if host or port still weren't specified, set default to local host and port 50051
    if host is None:
        host = 'localhost'
    if port is None:
        port = '50051'
    print(f"connecting to: {host}:{port}")
    # build client and run it
    client = BallVideoRTCClient(host, port, display=display, dp=dp, minDist=minDist)
    await client.run()


if __name__ == "__main__":
    # run client in an event loop
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Key interrupt")
