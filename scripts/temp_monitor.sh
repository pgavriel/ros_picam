#!/bin/bash

printf "%-15s%5s\n" "TIMESTAMP" "TEMP(Cel)"
printf "%20s\n" "---------------------"

while true
do
    temp=$(vcgencmd measure_temp | egrep -o '[0-9]*\.[0-9]*')
    timestamp=$(date +'%r')
    printf "%-15s%5s\n" "$timestamp" "$temp"
    sleep 10
done
