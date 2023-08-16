# written by Branden Kretschmer for Nimble Robotics python coding challenge
# email: brandenkretsch@gmail.com

# this file runs unit tests on client functions
from unittest import mock
import ctypes
import pytest
import multiprocessing
import numpy as np
import cv2 as cv
from client import client
import aiortc
import asyncio
from aiortc.contrib.signaling import TcpSocketSignaling, BYE

def test_find_center():
    '''
        Tests whether or not find center returns coordinates within 10 euclidean distance pixels from the actual center
    '''
    center = (400, 250)
    frame = np.zeros((480,640,3), dtype='uint8')
    frame = cv.circle(frame, center, 20, (255,255,255), -1) # detect center of white circle with radius of 20
    estimated_circle = client.detect_center(frame)[0:2]
    sq_diff = (np.array(center) - np.array(estimated_circle)) ** 2
    euclid_distance = np.sqrt(np.sum(sq_diff))
    assert euclid_distance < 5

# @mock.patch('client.detect_center', return_value )
def test_detect_center_proc():
    '''
        Tests that the detect_center_proc()
    '''
    que = multiprocessing.Queue()
    que.qsize = mock.Mock(return_value=1)
    que.get = mock.Mock(return_value=(1,1))
    val = multiprocessing.Value(client.POINT)
    val.x = 1
    val.y = 1
    val.time_stamp = -1
    cond = multiprocessing.Value(ctypes.c_int)
    cond.value = 0
    x = val.x
    y = val.y
    
    # test that values stay the same when detect_center returns None
    with mock.patch('client.client.detect_center', return_value = None, autospec=True):
        client.update_center_values(que, val, cond, 1, 1)
    
    assert val.x == x
    assert val.y == y
    assert val.time_stamp == 1
    assert cond.value == 1

    # test that values get updated when detect_center returns a tuple
    cond.value = 0
    x = 5
    y = 5
    with mock.patch('client.client.detect_center', return_value = (x,y,1), autospec=True):
        client.update_center_values(que, val, cond, 1, 1)
    
    assert val.x == x
    assert val.y == y
    assert val.time_stamp == 1
    assert cond.value == 1

    # test that nothing gets updated because cond value==1
    with mock.patch('client.client.detect_center', return_value = (1,1,1), autospec=True):
        client.update_center_values(que, val, cond, 1, 1)
    
    assert val.x == x
    assert val.y == y
    assert val.time_stamp == 1
    assert cond.value == 1


# Not sure how to make a test for functions where I have to mock an asynchronous coroutine and ran out of time

# @pytest.mark.asyncio
# async def test_RTCClient_consume_signal():
#     '''
#         Tests that consume signal returns the correct boolean depending on what is returned by signal.recv
#     '''
#     rtcclient = client.RTCClient('a', 'a')
    
#     # rtcclient.pc.setLocalDescription = mock.Mock(return_value=asyncio.coroutines)
#     # rtcclient.pc.setRemoteDescription = mock.Mock(return_value=1)
#     # rtcclient.pc.addIceCandidate = mock.Mock(return_value=1)
#     # rtcclient.signal.receive = mock.Mock(return_value=aiortc.RTCSessionDescription)
#     # RTCSD = aiortc.RTCSessionDescription()
#     with mock.patch('aiortc.contrib.signaling.TcpSocketSignaling.receive', return_value=aiortc.RTCSessionDescription('s', 'answer')):
#         s = await rtcclient.consume_signal()
#         # assert rtcclient.consume_signal()

#     # with mock.patch('aiortc.contrib.signaling.TcpSocketSignaling.receive', return_value=aiortc.RTCIceCandidate(1,1,1,1,1,1,1)):
#     #     assert rtcclient.consume_signal()

#     # with mock.patch('aiortc.contrib.signaling.TcpSocketSignaling.receive', return_value=BYE):
#     #     assert rtcclient.consume_signal()

# @pytest.mark.asyncio
# async def test_BallVideoRTCClient_run_track():
#     cli = client.BallVideoRTCClient('a', 'a')

#     cli._run_track()
   

