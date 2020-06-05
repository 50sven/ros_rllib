#!/bin/bash

while getopts 'c:' flag; do
    case "${flag}" in
        c) CONFIG=${OPTARG} ;;
        *) error "Unexpected option ${flag}" ;;
    esac
done

# Source config
source $CONFIG
ROS_MASTER_URI=http://ids-${ROS_MASTER}:${ROS_PORT}

function start_core () {

    export ROS_MASTER_URI=$1
    VENV_PATH=$(echo $4/bin/activate | sed 's;//;/;g')

    # Roscore
    tmux new -d -s Core_$2
    tmux send-keys -t Core_$2      "cd $3 &&
                                    export ROS_MASTER_URI=$1 &&
                                    source devel/setup.bash &&
                                    roscore -p $2" C-m

    cd $3 && source devel/setup.bash
    until dummy=$(rosnode info rosout | grep Pid); do sleep 1; done

    # Master node
    tmux new -d -s Master_$2
    tmux send-keys -t Master_$2    "cd $3 &&
                                    export ROS_MASTER_URI=$1 &&
                                    source ${VENV_PATH} &&
                                    source devel/setup.bash &&
                                    roslaunch ros_carla_rllib node_$5.launch type:=master" C-m
}

# Start roscore and master node
if [ ids-$ROS_MASTER == $HOSTNAME ]; then
    start_core $ROS_MASTER_URI $ROS_PORT $WORKSPACE $VENV $ALGORITHM
else
    ssh ids-${ROS_MASTER} << EOSSH
    $(declare -f start_core)
    start_core $ROS_MASTER_URI $ROS_PORT $WORKSPACE $VENV $ALGORITHM
EOSSH
fi
echo "Roscore and Master Node are running"
