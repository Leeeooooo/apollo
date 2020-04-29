#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd "${DIR}/.."

source "${DIR}/apollo_base.sh"

function start() {
    INS_NUM_PROCESSES="$(pgrep -c -f "ins_stat_publisher")"
    ODO_NUM_PROCESSES="$(pgrep -c -f "odom_publisher")"
    INS_LOG="${APOLLO_ROOT_DIR}/data/log/ins_publisher.out"
    ODO_LOG="${APOLLO_ROOT_DIR}/data/log/odom_publisher.out"
    if [ "${INS_NUM_PROCESSES}" -eq 0 ]; then
        nohup python /apollo/modules/tools/sensor_calibration/ins_stat_publisher.py </dev/null >${INS_LOG} 2>&1 &
    fi
    if [ "${ODO_NUM_PROCESSES}" -eq 0 ]; then
        nohup python /apollo/modules/tools/sensor_calibration/odom_publisher.py </dev/null >${ODO_LOG} 2>&1 &
    fi
}

function stop() {
  pkill -SIGINT -f odom_publisher
  pkill -SIGINT -f ins_stat_publisher
}

case $1 in
  start)
    shift
    start $@
    ;;
  stop)
    shift
    stop $@
    ;;
  *)
    start $@
    ;;
esac
