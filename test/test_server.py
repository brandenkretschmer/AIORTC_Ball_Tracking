# written by Branden Kretschmer for Nimble Robotics python coding challenge
# email: brandenkretsch@gmail.com

# this file runs unit tests on server functions

import pytest
import os
import sys
cwd = os.getcwd()
print(cwd)
sys.path.append(cwd)
import server.server as server
import client.client as client


