#!/bin/bash

if [ -z "$1" ]; then
  echo "Usage: $0 <hours>"
  exit 1
fi

hours=$1
end_time=$(($(date +%s) + hours * 3600))

log_file="command_execution_log.txt"
echo "Command execution log - $(date)"
echo "Command execution log - $(date)" > $log_file
echo "Executing for $hours hours" 
echo "Executing for $hours hours" >> $log_file

start_carla_script="cd ~/apollo_carla_8/carla_apollo_bridge/select_by_user_name_scripts/ && ./run_carla_baseline.sh"

while [ $(date +%s) -lt $end_time ]; do
  if docker ps | grep -q carla-cfs-bl; then
    start_time=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$start_time] Carla container is already running" >> $log_file
  elif docker ps -a | grep -q carla-cfs-bl; then
    start_time=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$start_time] Starting stopped Carla container" >> $log_file
    docker start carla-cfs-bl
    sleep 10
  else
    start_time=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$start_time] Carla container not found, starting with script" >> $log_file
    eval $start_carla_script
    sleep 10
  fi


  if docker ps | grep -q apollo_dev_bl8889; then
    start_time=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$start_time] Starting test program"
    echo "[$start_time] Starting test program" >> $log_file
    docker exec -it -u bl8889 -e HISTFILE=/apollo/.dev_bash_hist apollo_dev_bl8889 /bin/bash -i -c 'cd /apollo/modules/MS_fuzz/base_line/random && python random_scene_town04.py --town 4 -p 4000'
  else
    start_time=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$start_time] Apollo dev container not running, skipping test program" >> $log_file
    exit 1
  fi

  sleep 3
done

echo "Execution completed at $(date)"
echo "Execution completed at $(date)" >> $log_file
