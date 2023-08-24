# AIORTC Client and Server
Repository Written by Branden Kretschmer  email: brandenkretsch@gmail.com  

### Note: This repository was developed on an apple silicon mac

### Dependencies
All Python code written in this repository was tested with Python 3.11.4 on an apple silicon mac. Dependencies for this repository can be found in  the requirements.txt file or the conda_environment.yaml. The docker files for creating client and server images use Ubuntu 22.04 as their base  and then install Python 3.11.4 and all dependencies outlined in requirements.txt (view client and server docker files for more info)


## How to Run Tests
just run ```pytest``` as a command in the root directory of this repository


## How to Run Python Scripts
Spin up a Python 3.11 environment with all the dependencies required. This can be done with either conda or pip. 
Once the requirements have been installed, make 2 calls from the root directory of this repository:  
1. ```python server/server.py -n localhost -p 50051 -d```  
2. ```python client/client.py -n localhost -p 50051 -d --dp 6```  
Two opencv images should appear. RMSE between estimated and actual center is shown in the server's output log

## How to Run in KIND (Kubernetes in Docker)
make shure kind, docker, and kubectl are installed  
1. Build the docker images using ```./build_docker.sh```  
2. Create a kind cluster using command: ```kind create cluster --name test```  
3. Point kubectl to the newly creaetd cluster: ```kubectl cluster-info --context kind-test```  
4. Set up the cluster and apply deployments and service: ```setup_cluster.sh```
5. Look at logs of server to find the RMSE of the estimated circle center.
6. Cleanup cluster when finished

## What Could Improve?
More unit tests for functions, especially the asynchronous ones (I ran out of time to figure out how to mock a coroutine and write tests with the mock)
Better interface definitions for RTCClient and RTCServer classes (ex. have them inherit from an ABC)  
Handle multiple connections to the server at once  
Better logging instead of using print to log things. This would help with the k8s deployment  
At higher resolutions, things crash due to latency in frame processing from both the client side and the server side.  The latency causes things to hang. Scaling frame rate inversely with resolution would solve this problem. So would using more processes and/or posix threads.  
Deployment files could be scaled up, but the server-signal-service would have to become a load balancer that only send 1 connection at a time to a server (server can only handle one connection).  
