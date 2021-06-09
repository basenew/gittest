CRTDIR=$(pwd)
TINY_ROS_MSGS=$(pwd)/../
TINY_ROS_CORE=$(pwd)/../src/middleware/tinyros
TINY_ROS_OUTPUT=$(pwd)/../devel/include/tinyros_client_library/rtthread
TINY_ROS_LIBRARY=$(pwd)/tinyros_client_library 

python ${TINY_ROS_CORE}/scripts/make_library_rtthread.py ${TINY_ROS_MSGS} ${TINY_ROS_OUTPUT} ${TINY_ROS_CORE}
rm -rf ${TINY_ROS_LIBRARY}
cp -a ${TINY_ROS_OUTPUT} ${TINY_ROS_LIBRARY}
if [ $? != 0 ];then
    exit 1
fi

echo build mcu

cd ${CRTDIR}/monitor
./build.sh $1
if [ $? != 0 ];then
    exit 1
fi

cd ${CRTDIR}/chassis_controller
./build.sh $1
if [ $? != 0 ];then
    exit 1
fi

cd ${CRTDIR}/motor_controller
./build.sh $1
if [ $? != 0 ];then
    exit 1
fi



