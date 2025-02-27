#!/bin/bash

run_option=$*

source ./install/setup.bash

ros2 run --prefix 'gdbserver localhost:3038' $run_option 
# if [ -f /usr/bin/gnome-terminal ]
# then
#     gnome-terminal -- bash -c " ros2 run --prefix 'gdbserver localhost:3038' $run_option"
#     # gnome-terminal -- bash -c " ros2 run --prefix 'gdb -ex run --args' $run_option"
# else
#     if [ -f /usr/bin/konsole ]
#     then
#         konsole -e "ros2 run --prefix  'gdbserver localhost:3038' $run_option"
#     fi
# fi
