

project_path=$(cd `dirname $0`; pwd)
project_name="${project_path##*/}"
echo "=== build ${project_name} ==="
echo $project_path
echo $project_name
echo "=== preprocessor ==="
   python ${project_path}/../preprocessor.py ${project_path}/ubt_common.h

echo build jumper...
cd jumper

if [ -n "$1" ]; then        
 make clean
else
   echo "===> make jumper ==>"
   make clean
   make
   if [ $? -ne 0 ]; then
      echo project $project_name
      echo "make jumper failed!"
      exit 1
   fi
fi


cd ..

echo build bootloader
cd bootloader
scons -c
scons $1

if [ $? -ne 0 ]; then
      echo project $project_name
      echo "make bootloader failed!"
   exit 1
fi

cd ..

echo build APP
cd APP
scons -c
scons $1

if [ $? -ne 0 ]; then
      echo project $project_name
      echo "make APP failed!"
   exit 1
fi

cd ../..
if [ ! -n "$1" ];then
   echo "===> now linker ${project_name} jumper + bootloader + app ==> burn.bin"


   if [ -d "${project_path}/../bin/${project_name}" ];then
      echo delete old bin
      rm ${project_path}/../bin/${project_name}/*; 
   else
     echo no find bin creat bin
     mkdir -p ${project_path}/../bin/${project_name}
   fi
   cp ${project_path}/jumper/jumper.bin ${project_path}/../bin/${project_name}/
   cp ${project_path}/bootloader/bootloader.bin ${project_path}/../bin/${project_name}/
   cp ${project_path}/APP/app.bin  ${project_path}/../bin/${project_name}/
   cd ${project_path}/../bin/${project_name}/
   python ${project_path}/../merge.py ${project_path}/ubt_common.h ${project_path}/../bin/${project_name}/
   python ${project_path}/../head_package.py ${project_path}/ubt_common.h ${project_path}/../bin/${project_name}/app.bin app.rbl app
   python ${project_path}/../head_package.py ${project_path}/ubt_common.h ${project_path}/../bin/${project_name}/bootloader.bin bootloader.rbl bl
   python ${project_path}/../gen_config.py ${project_path}/ubt_common.h ${project_name} ${project_path}/../bin/${project_name} config.yaml app
else
   cd ${project_path}/../bin/${project_name}/
   rm *.bin  
fi

