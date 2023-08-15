# AIORTC Client and Server
Repository Written by Branden Kretschmer  email: brandenkretsch@gmail.com
### Note: This repository was developed on an apple silicon mac

### Dependencies
All Python code written in this repository was tested with Python 3.11.4 on an apple silicon mac. Dependencies for this repository can be found in  the requirements.txt file or the conda_environment.yaml. The docker files for creating client and server images use Ubuntu 22.04 as their base  and then install Python 3.11.4 and all dependencies outlined in requirements.txt (view client and server docker files for more info)


## Server Design
###

## Client Design
### Opencv Circle Detection

## How to Run Python Scripts
Spin up a Python 3.11 environment with all the dependencies required. This can be done with either conda or pip. 
Once the requirements have been installed, make 2 calls from the root directory of this repository:  
1. ```python server/server.py -n localhost -p 50051 -d```  
2. ```python client/client.py -n localhost -p 50051 -d --dp 6```  
Two opencv images should appear

## How to Run in KIND (Kubernetes in Docker)
make shure kind, docker, and kubectl are installed  
First, build the docker images using ```./build_docker.sh```  
Then create a kind cluster using command: ```kind create cluster --name test```
Point kubectl to the newly creaetd cluster: ```kubectl cluster-info --context kind-test```

Alternatively, just run the build_docker.sh script, build_kind_cluster.sh script, and the run_in_kind.sh script to get the demo going.  Then access the pod logs to see what they are printing.  
Note that when running in kubernetes, no images can be displayed, only logs from the client and server can be seen.

## What Could Improve?
More unit tests for functions  
Better interface definitions for RTCClient and RTCServer classes (ex. have them inherit from an ABC)  
Multiple connections to the server at once  
Better logging instead of using print to log things. This would help with the k8s deployment  
At higher resolutions, things crash due to latency in frame processing from both the client side and the server side.  The latency causes things to hang. Scaling frame rate inversely with resolution would solve this problem. So would using more processes/posix threads.
