# Written by Branden Kretschmer

# load docker files
kind load docker-image rtcserver:1 --name test
kind load docker-image rtcclient:1 --name test

# start deployments on the default namespace
kubectl apply -f ./server/k8s/rtcserver_service.yml
kubectl apply -f ./server/k8s/rtcserver_deployment.yml
kubectl apply -f ./client/k8s/rtcclient_deployment.yml
