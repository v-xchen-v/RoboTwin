build:
```
sudo docker build -t robotwin:latest -f docker/Dockerfile .
```

run container:
```
xhost +
docker run -it --rm \
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ${PWD}:/workspace \
  --name robotwin \
  robotwin:latest \
  /bin/bash
  ```