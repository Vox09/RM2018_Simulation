#!/bin/bash
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(pwd)/../plugins">>~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/../meshes">>~/.bashrc
echo "set up successully"
