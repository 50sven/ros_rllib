#!/bin/bash

while getopts 'c:i:' flag; do
    case "${flag}" in
        c) CONFIG=${OPTARG} ;;
        i) INDEX=${OPTARG} ;;
        *) error "Unexpected option ${flag}" ;;
    esac
done

# Source config
source $CONFIG
ROS_MASTER_URI=http://ids-${ROS_MASTER}:${ROS_PORT}
HOST=ids-${HOSTS[$INDEX]}
PORT=${PORTS[$INDEX]}
MAP=${MAPS[$INDEX]}
SCENARIO=${SCENARIOS[$INDEX]}

function start_env () {

    VENV_PATH=$(echo $3/bin/activate | sed 's;//;/;g')

    # Start Carla Session
    tmux new -d -s Carla_$9
    tmux send-keys -t Carla_$9     "cd ${CARLA_ROOT} &&
                                    DISPLAY= ./CarlaUE4.sh -opengl -carla-rpc-port=$5" C-m

    # Wait for Carla to start loading
    until dummy=$(nvidia-smi | grep 'CarlaUE4' | awk '{print $4}'); do sleep 1; done

    # TBD: Proper check for running carla sever
    # Current procedure checks for any carla server
    # Sleep 30 prevents from early continuation
    while :
    do
        TYPE=$(nvidia-smi | grep 'CarlaUE4' | awk '{print $4}')
        if [ "$TYPE" == "C+G" ]
        then
            break
        else
            sleep 1
        fi
    done
    sleep 30

    # Start Environment Session
    tmux new -d -s Env_$9
    tmux send-keys -t Env_$9       "cd $2 &&
                                    export ROS_MASTER_URI=$1 &&
                                    source ${VENV_PATH} &&
                                    source devel/setup.bash &&
                                    roslaunch ros_carla_rllib node_$8.launch type:=env index:=$4 port:=$5 map:=$6 scenario:=$7" C-m
}

if [ $HOST == $HOSTNAME ]; then
    start_env $ROS_MASTER_URI $WORKSPACE $VENV ${HOSTS[$INDEX]} $PORT $MAP $SCENARIO $ALGORITHM $ROS_PORT
else
    ssh -X $HOST << EOSSH
    $(declare -f start_env)
    start_env $ROS_MASTER_URI $WORKSPACE $VENV ${HOSTS[$INDEX]} $PORT $MAP $SCENARIO $ALGORITHM $ROS_PORT
EOSSH
fi
echo "Carla and Env are running on $HOST"
