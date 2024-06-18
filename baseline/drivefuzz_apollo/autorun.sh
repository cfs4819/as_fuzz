#!/bin/bash
fuzzerdata_dir="/tmp/fuzzerdata/"
t=86400  
retry_time=0
start_time=$(date +%s)  

autorun_path=/apollo/data/drivefuzz/24h_Town04/result_autorun_"$(date +%Y%m%d_%H%M%S)"

mkdir -p $autorun_path

while true; do
    # Create fuzzerdata_dir if it doesn't exist
    if [[ ! -d "$fuzzerdata_dir" ]]; then
        mkdir -p "$fuzzerdata_dir"
        echo "Created directory $fuzzerdata_dir"
    fi
    
    # docker kill carla-chenpansong-bl

    sleep 1
    current_time=$(date +%s)
    total_duration=$((current_time - start_time))
    if [ $total_duration -ge $t ]; then
        echo "Total duration exceeded $t seconds. Exiting..."
        break
    fi

    python ./fuzzer.py -o "$autorun_path/output_$retry_time" -s seed1 --town 4
    retry_time=$((retry_time+1))
done


