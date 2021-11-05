# Perception Info

## Setup Using (Deepo)[https://github.com/ufoym/deepo]

Deepo should be sufficient for now and getting started. Here are getting started steps (not tested yet)

1. Pull Deepo docker images with `docker pull ufoym/deepo`

2. Run Deepo docker image with `docker run --gpus all -it -v <local directory>:<desired directory mounting location in container> --ipc=host -p 8888:8888 ufoym/deepo bash` 
