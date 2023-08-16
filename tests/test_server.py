# written by Branden Kretschmer for Nimble Robotics python coding challenge
# email: brandenkretsch@gmail.com

# this file runs unit tests on server functions

import pytest
from unittest import mock
import ctypes
import pytest
import multiprocessing
import numpy as np
import cv2 as cv
from server import server
import aiortc
import asyncio
from aiortc.contrib.signaling import TcpSocketSignaling, BYE

def test_FrameGenerator():
    '''
        tests frame generator in server
    '''
    # test start location
    frame_gen = server.FrameGenerator(5, 40, (640,480))
    assert frame_gen.get_current_location() == (40, 40)

    # test velocity
    for _ in range(3):
        frame_gen.increment_position()
    assert frame_gen.get_current_location() == (55, 55)

    # test y bounce
    point = 55
    while point + 40 < 480 + 5: # 40 is the radius of the circle
        frame_gen.increment_position()
        point += 5
    assert frame_gen.y_sense == -1
    assert frame_gen.x_sense == 1

    # test x bounce
    while point + 40 < 640 + 4:
        frame_gen.increment_position()
        point += 5
    assert frame_gen.y_sense == -1
    assert frame_gen.x_sense == -1

    

