#!/bin/sh
process_name=$1
config_file=$2

# get xxx（xxx.json）  
config_basename=$(basename $config_file)  
# delete .json  
config_name=${config_basename%.json}  

output_file="log_memory_${config_name}.txt"

# run the process
# kill possible existing process  
pkill $process_name
$process_name $config_file &
main_pid=$!

##ps aux | grep $process_name
echo $process_name $main_pid >>$output_file
echo "Monitor process script running..."
while true
do  
  # check if the process is still running
  if ! kill -0 $main_pid 2>/dev/null; then  
    echo "The process $process_name has ended. Stopping monitor..."  
    break  
  fi  
  
  # monitor the memory usage of the process
  ps aux | awk -v pid=$main_pid '$2 == pid {printf "VSZ,%s,RSS,%s\n", $5, $6}' | tee -a $output_file  
  
  sleep 0.1  
done
echo "Monitor script has exited."