

project_path=$(cd `dirname $0`; pwd)
project_name="${project_path##*/}"
echo "=== build ${project_name} ==="
echo $project_path
echo $project_name
echo "=== preprocessor ==="
   python ${project_path}/../../html_toC_file.py ${project_path}/chassis/httpd/html/ ${project_path}/chassis/httpd/fsdata_custom.h

