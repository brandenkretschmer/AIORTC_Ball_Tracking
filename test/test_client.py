# written by Branden Kretschmer for Nimble Robotics python coding challenge
# email: brandenkretsch@gmail.com

# this file runs unit tests on client functions

import pytest
import os
import sys
cwd = os.getcwd()
print(cwd)
sys.path.append(cwd)
import client.client as client
import server.server as server