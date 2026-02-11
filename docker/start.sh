#!/bin/bash

# Start container if it's not already running
docker start graspgen_ros1_container > /dev/null

# Attach into it
docker exec -it graspgen_ros1_container bash