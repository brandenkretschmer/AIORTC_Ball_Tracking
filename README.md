Repository Written by Branden Kretschmer
email: brandenkretsch@gmail.com
### Note: This repository was developed on an apple silicon mac

### Dependencies
All Python code written in this repository was tested with Python 3.11.4 on an apple silicon mac. Dependencies for this repository can be found in  the requirements.txt file or the conda_environment.yaml. The docker files for creating client and server images use Ubuntu 22.04 as their base  and then install Python 3.11.4 and all dependencies outlined in requirements.txt (view client and server docker files for more info)

# AIORTC Client and Server

## Server Design

## Client Design

## How to run scripts
Spin up a Python 3.11 environment with all the dependencies required. This can be done with either conda or pip. 

## How to run in kind (kubernetes in docker)
Alternatively, just run the build_docker.sh script, build_kind_cluster.sh script, and the run_in_kind.sh script to get the demo going.  Then access the pod logs to see what they are printing.
