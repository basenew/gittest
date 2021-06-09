#########################################################################
# File Name: appAutoRun.sh
# Created Time: 2018-03-24 11:05:41
#########################################################################
#!/bin/bash

atris_start_daemon()
{
    while true; do

        sleep 1

        result=`busybox ps|grep "roscore"|grep -v grep`
        if [ -z "${result}" ];then
            roscore &
        fi

        sleep 1

        result=`busybox ps|grep "lib/tinyros/tinyrosdds"|grep -v grep`
        if [ -z "${result}" ];then
            /home/atris/atris_app/lib/tinyros/tinyrosdds &
        fi

        sleep 1

        result=`busybox ps|grep "lib/tinyros/tinyrosconsole"|grep -v grep`
        if [ -z "${result}" ];then
            /home/atris/atris_app/lib/tinyros/tinyrosconsole -d 0 /userdata/atris_app/logs &
        fi

        sleep 1

        result=`busybox ps|grep "lib/diagnostics/diagnostics"|grep -v grep`
        if [ -z "${result}" ];then
            /home/atris/atris_app/lib/diagnostics/diagnostics &
        fi

        sleep 1

        result=`busybox ps|grep "lib/devices/devices"|grep -v grep`
        if [ -z "${result}" ];then
            LD_LIBRARY_PATH=/home/atris/atris_app/lib/ffmpeg:/home/atris/atris_app/lib/HCNetSDKCom:/opt/ros/melodic/lib:/home/atris/atris_app/lib /home/atris/atris_app/lib/devices/devices &
        fi

        sleep 1

        result=`busybox ps|grep "lib/navability/navability"|grep -v grep`
        if [ -z "${result}" ];then
            /home/atris/atris_app/lib/navability/navability &
        fi

        sleep 1

        result=`busybox ps|grep "lib/navigation/navigation"|grep -v grep`
        if [ -z "${result}" ];then
            /home/atris/atris_app/lib/navigation/navigation &
        fi

        sleep 1

        result=`busybox ps|grep "lib/udock/udock"|grep -v grep`
        if [ -z "${result}" ];then
            /home/atris/atris_app/lib/udock/udock &
        fi

        sleep 1

        result=`busybox ps|grep "lib/shttpd/shttpd"|grep -v grep`
        if [ -z "${result}" ];then
            /home/atris/atris_app/lib/shttpd/shttpd -root /home/atris/atris_app/bin/dist &
        fi

        sleep 1

        result=`busybox ps|grep "lib/mosquitto/mosquitto"|grep -v grep`
        if [ -z "${result}" ];then
            /home/atris/atris_app/lib/mosquitto/mosquitto -c /home/atris/atris_app/config/mosquitto/mosquitto.conf &
        fi

        sleep 1

        result=`busybox ps|grep "lib/aisound/aisound"|grep -v grep`
        if [ -z "${result}" ];then
            /home/atris/atris_app/lib/aisound/aisound &
        fi

        sleep 1

        result=`busybox ps|grep "lib/swupgrade/swupgrade"|grep -v grep`
        if [ -z "${result}" ];then
            /home/atris/atris_app/lib/swupgrade/swupgrade &
        fi

        sleep 1

        result=`busybox ps|grep "lib/signaling/signaling"|grep -v grep`
        if [ -z "${result}" ];then
            /home/atris/atris_app/lib/signaling/signaling &
        fi

        sleep 1

        result=`busybox ps|grep "lib/visionability/visionability"|grep -v grep`
        if [ -z "${result}" ];then
            LD_LIBRARY_PATH=/home/atris/atris_app/lib/grpclib:/opt/ros/melodic/lib:/home/atris/atris_app/lib /home/atris/atris_app/lib/visionability/visionability &
        fi
    done
}

# ros env
export LD_LIBRARY_PATH=/home/atris/atris_app/lib:/opt/ros/melodic/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/home/atris/atris_app:/opt/ros/melodic:$CMAKE_PREFIX_PATH
export PYTHONPATH=/home/atris/atris_app/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages:$PYTHONPATH
export ROS_PACKAGE_PATH=/home/atris/atris_app/share:/opt/ros/melodic/share:$ROS_PACKAGE_PATH
export PATH=/home/atris/atris_app/bin:/opt/ros/melodic/bin:$PATH
export PKG_CONFIG_PATH=/home/atris/atris_app/lib/pkgconfig:/opt/ros/melodic/lib/pkgconfig:$PKG_CONFIG_PATH
export ROS_DISTRO=melodic
rm -rf /root/.ros/log
rm -rf /home/atris/.ros/log

# remove tmp files
sudo mkdir -p /userdata/tmp
if [ -d /userdata/tmp ]; then
    rm -rf /userdata/tmp/*
    sudo mkdir --mode=0777 /userdata/tmp/maps
fi

# DNS server
echo "nameserver 10.20.18.1" > /etc/resolv.conf
echo "nameserver 119.29.29.29" >> /etc/resolv.conf
echo "nameserver 114.114.114.114" >> /etc/resolv.conf
echo "nameserver 1.2.4.8" >> /etc/resolv.conf

mkdir -p /userdata/atris_app/schemes

# set core_pattern
mkdir -p /userdata/atris_app/logs
chmod -R 0777 /userdata/atris_app/logs
echo "/userdata/atris_app/logs/core-%e-%p-%t" > /proc/sys/kernel/core_pattern

# core file size
# 1K-blocks: 8192 blocks
ulimit -c unlimited

# hfs
rm /home/atris/atris_app/bin/dist/hfs
ln -s /userdata/tmp /home/atris/atris_app/bin/dist/hfs
sudo mkdir -p /userdata/tmp/img
sudo mkdir -p /userdata/tmp/video
sudo mkdir -p /userdata/tmp/audio
sudo mkdir -p /userdata/tmp/rp
# Restore from upgrade backup
atris_app_path=/home/atris/atris_app
atris_app_bak_path=/home/atris/.atris_app_bak
if [ -d ${atris_app_bak_path} ]; then
    if [ ! -d ${atris_app_path} ]; then
        mv ${atris_app_bak_path} ${atris_app_path}
    else
        rm -rf ${atris_app_bak_path}
    fi
fi

# Start applications
atris_start_daemon &

