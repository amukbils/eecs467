#!/bin/bash

if [ $# -eq 0 ]
then 
	echo "No destination provided. Expected use: ./sync_code.sh teamN@maebot_ip"
else
	rsync -pvrzt --progress --exclude "bin/*" --exclude "*.o" --exclude "*.a" * $1:eecs467/
fi
