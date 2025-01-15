#!/bin/bash
APOLLO_PATH=$(dirname "$(dirname "$(pwd)")")

if [ -z "$1" ]; then
  echo "Usage: $0 <hours>"
  exit 1
fi

hours=$1
end_time=$(($(date +%s) + hours * 3600))
apollo_container_name="apollo_dev_$USER"
carla_container_name="carla-$USER"

log_file="autorun_results/command_execution_log.txt"
echo "Command execution log - $(date)"
echo "Command execution log - $(date)" > $log_file
echo "Executing for $hours hours" 
echo "Executing for $hours hours" >> $log_file

start_carla_script="$APOLLO_PATH/modules/MS_fuzz/run_carla_offscreen.sh"
run_cmd='cd /apollo/modules/MS_fuzz/ && python msfuzz.py --town 10 -p 5000'

while [ $(date +%s) -lt $end_time ]; do
  if docker ps | grep -q $carla_container_name; then
    start_time=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$start_time] Carla container is already running"
    echo "[$start_time] Carla container is already running" >> $log_file
  elif docker ps -a | grep -q $carla_container_name; then
    start_time=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$start_time] Starting stopped Carla container"
    echo "[$start_time] Starting stopped Carla container" >> $log_file
    docker start $carla_container_name
    sleep 10
  else
    start_time=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$start_time] Carla container not found, starting with script"
    echo "[$start_time] Carla container not found, starting with script" >> $log_file
    $start_carla_script
    sleep 10
  fi


  if docker ps | grep -q $apollo_container_name; then
    start_time=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$start_time] Starting test program"
    echo "[$start_time] Starting test program" >> $log_file
    docker exec -it -u $USER -e HISTFILE=/apollo/.dev_bash_hist $apollo_container_name /bin/bash -i -c "$run_cmd"
  else
    start_time=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$start_time] Apollo dev container not running, skipping test program"
    echo "[$start_time] Apollo dev container not running, skipping test program" >> $log_file
    exit 1
  fi

  sleep 3
done

echo "Execution completed at $(date)"
echo "Execution completed at $(date)" >> $log_file
