#!/bin/bash

CONFIG=${HOME}/no_backup/catkin_ws/src/ros_carla_rllib/scripts/setting.config

while getopts 'c' flag; do
    case "${flag}" in
        c) CONFIG=${OPTARG} ;;
        *) error "Unexpected option ${flag}" ;;
    esac
done

export ROS_PORT=${ROS_PORT}

cleanup() {

for index in ${!HOSTS[*]} ; do {
    echo "Closing tmux server on ids-${HOSTS[$index]}"
    ssh ids-${HOSTS[$index]} <<EOSSH
    tmux kill-session -t Env_${ROS_PORT}
    sleep 0.25
    tmux kill-session -t Carla_${ROS_PORT}
    sleep 0.25
EOSSH
} done

echo "Closing tmux server on ROS_MASTER"
ssh ids-${ROS_MASTER} <<EOSSH
    tmux kill-session -t Eval_${ROS_PORT}
    sleep 0.25
    tmux kill-session -t Master_${ROS_PORT}
    sleep 0.25
    tmux kill-session -t Core_${ROS_PORT}
    sleep 0.25
    tmux kill-session -t Carla_${ROS_PORT}
    sleep 0.25
EOSSH
exit 1
}

trap cleanup 1 2 3 6

# Source config
source $CONFIG
DIR=$(dirname "$(readlink -f "$0")")

# Roscore and Master Node
echo "Starting Roscore and Master Node on ids-${ROS_MASTER}:${ROS_PORT}"
bash $DIR/startCore.sh -c $CONFIG
echo "Starting Eval Node on ids-${EVAL_HOST}:${EVAL_PORT}"
bash $DIR/startEval.sh -c $CONFIG &

# Carla and Env Nodes
for index in ${!HOSTS[*]} ; do {
    echo "Starting Carla-Env on ids-${HOSTS[$index]}:${PORTS[$index]}" &&
    bash $DIR/startEnv.sh -c $CONFIG -i $index &
} done

wait

while :
do
    echo "Training is running..."
    sleep 60
done
