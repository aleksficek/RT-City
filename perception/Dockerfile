# get image from https://hub.docker.com/r/nvidia/cuda/tags for relevant repo 
# (change CUDA/cuDNN versions as needed)
FROM nvidia/cuda:10.2-cudnn8-devel-ubuntu18.04 


ENV NVIDIA_VISIBLE_DEVICES all 
ENV NVDIIA_DRIVER_CAPABILITIES all 

ENV HOME /host-machine 

# should get basic Ubuntu/bash utilities
RUN apt-get update \ 
	&& apt-get install -y software-properties-common 	\
	&& add-apt-repository pp:deadsnakes/ppa	            \
	&& apt-get update                               	\
	&& apt-get install -y python3.6 	                \
	&& apt-get install -y python3-pip                	\
	&& apt-get install -y libsm6 libxext6           	\
	&& apt-get install -y libxrender-dev python-future 	\
	&& apt-get install -y libgl1-mesa-dev           	\
	&& apt-get install -y vim	                        \
	&& apt-get update                                   \
	&& apt-get install -y screen                     	\
	&& apt-get install -y wget                       	\
	&& apt-get install -y sudo                      	\
	&& apt-get install -y git                    

# customize these Python packages based on the repo you are using
RUN python3 -m pip install --upgrade pip                \
	&& python3 -m pip install numpy                     \
	&& python3 -m pip install matplotlib                \
	&& python3 -m pip install PyYAML                    \
	&& python3 -m pip install torch==1.8.0              \
	&& python3 -m pip install Cython                    

CMD /bin/bash 
