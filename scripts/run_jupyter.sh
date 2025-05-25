#!/bin/bash
docker exec -it ros-melodic bash -c "source /opt/ros/melodic/setup.bash && jupyter notebook --ip=0.0.0.0 --port=8888 --allow-root --NotebookApp.token=''"