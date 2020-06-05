while getopts 'c:' flag; do
    case "${flag}" in
        c) CONFIG=${OPTARG} ;;
        *) error "Unexpected option ${flag}" ;;
    esac
done

# Source config
source $CONFIG
ROS_MASTER_URI=http://ids-${ROS_MASTER}:${ROS_PORT}

function start_eval () {

    VENV_PATH=$(echo $3/bin/activate | sed 's;//;/;g')

    # Start Carla Session
    tmux new -d -s Carla_$7
    tmux send-keys -t Carla_$7     "cd ${CARLA_ROOT} &&
                                    DISPLAY= ./CarlaUE4.sh -opengl -carla-rpc-port=$4" C-m

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

    # Start Evaluation Session
    tmux new -d -s Eval_$7
    tmux send-keys -t Eval_$7      "cd $2 &&
                                    export ROS_MASTER_URI=$1 &&
                                    source ${VENV_PATH} &&
                                    source devel/setup.bash &&
                                    roslaunch ros_carla_rllib node_$6.launch type:=eval port:=$4 map:=$5" C-m
}

if [ $EVAL_HOST == $HOSTNAME ]; then
    start_eval $ROS_MASTER_URI $WORKSPACE $VENV $EVAL_PORT $EVAL_MAP $ALGORITHM $ROS_PORT
else
    ssh -X ids-$EVAL_HOST << EOSSH
    $(declare -f start_eval)
    start_eval $ROS_MASTER_URI $WORKSPACE $VENV $EVAL_PORT $EVAL_MAP $ALGORITHM $ROS_PORT
EOSSH
fi
echo "Carla and Eval are running on $EVAL_HOST"
