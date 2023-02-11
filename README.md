# Setup

Just run
```
docker build -t ros_handson .
```

# Run docker image

```bash
docker run --rm -p 6080:80 --shm-size=512m --security-opt seccomp=unconfined ros_handson
```

Browse [http://127.0.0.1:6080/](http://127.0.0.1:6080/)