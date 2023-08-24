# utility script to build docker files
# note file structure is important here. Don't move the files

docker build -f server/docker/Dockerfile.server -t rtcserver:1 .
docker build -f client/docker/Dockerfile.client -t rtcclient:1 .
