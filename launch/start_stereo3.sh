#!/bin/bash
#检查程序是否已经在运行
#if pgrep -f "stereo3" >/dev/null; then 
#  echo "程序已经在运行"
#else
  #启动程序的命令
#  /root/dev/stereo3_ros1_ws/install/lib/stereo3/stereo3 &
#  echo "程序已启动"
#fi

#while [ true ]
#do
#  /root/dev/stereo3_ros1_ws/install/lib/stereo3/stereo3 &
#done  


#PIDS=`ps -ef |grep stereo3 |grep -v grep | awk '{print $2}'`
PIDS=`pgrep -f stereo3`
echo $PIDS

while [ 1 > 0 ]
do
#echo "stereo3 is runing!"
if [ "$PIDS" != "" ]; then
#echo "stereo3 is runing!"
#else
#cd /root/
#./stereo3
#运行进程
fi
done
