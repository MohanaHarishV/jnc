#!/bin/bash

sleep $1

shift # The sleep time is droped

ros2 launch $@