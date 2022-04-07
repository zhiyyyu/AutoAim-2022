#!/bin/bash

sec=5
cnt=0
path=/home/zhiyu/AIM_HUST/AutoAim/cmake-build-debug/
name=AutoAim
cd $path
# make clean && make -j
while [ 1 ]
do
count=`ps -ef | grep $name | grep -v "/home" | grep -v "grep" | wc -l`
thread=`ps -ef | grep $name | grep -v "/home" | grep -v "grep"`
echo $thread
echo "Thread count: $count"
echo "Expection count: $cnt"
if [ $count -gt 0 ]; then
	echo "The $name is still alive!"
	sleep $sec
else 
	echo "Starting $name..."
	chmod +x /dev/ttyUSB0

   	cd $path
   	./$name	&
   	echo "$name has started!"		
	sleep $sec

	#cnt=$((cnt+1))
	#if [ $cnt -gt 9 ]; then
	#	reboot
	#fi
fi
done
