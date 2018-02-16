#!/usr/bin/env bash
# packages should use rosdep dependencies
# what is not in rosdep should go here

# art_projected_gui
sudo apt install liblapack-dev liblas-dev gfortran
sudo pip install transitions qimage2ndarray enum34 scipy matplotlib playsound
