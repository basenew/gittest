#!/bin/sh

echo -e "\nota_pepare: pepare update the system..."
echo -e "---------------------------------"

# tar file path
path_tar_file=$1

path_swupdate=/swupdate
path_swupdate_temp=/swupdate/tmp
target_path=/home/atris/atris_app
target_path_bak=/home/atris/.atris_app_bak
target_path_tmp=/home/atris/.atris_app_tmp
mkdir -p ${path_swupdate_temp}
mkdir -p ${target_path}
mkdir -p ${target_path_tmp}
rm -rf ${path_swupdate_temp}/*
rm -rf ${target_path_tmp}/*
rm -rf ${target_path_bak}

xs_nav_target_path=/home/atris/xs_nav
xs_pack_file_path=/home/atris/atris_app/firmware/xsnav
xs_pack_file=${xs_pack_file_path}/*.tar.gz
xs_pack_file_md5sum=${xs_pack_file_path}/*.md5
mkdir -p ${xs_nav_target_path}
chown -R atris ${xs_nav_target_path}
chgrp -R atris ${xs_nav_target_path}

upgrade_setenv() {
    file="/etc/atris_system_env.ini"
    if [ ! -f ${file} ]; then
        echo "recovery_status=success" > ${file}
    fi
    val=$1
    sed -i "s/\(recovery_status=\).*/\1${val}/g" ${file}
}

upgrade_printenv() {
    file="/etc/atris_system_env.ini"
    if [ ! -f ${file} ]; then
        echo "recovery_status=success" > ${file}
    fi
    val=$(awk -F '=' '("recovery_status"==$1){print $2}' ${file}) 
    echo -e "${val}"
}

upgrade_progress() {
    upgrade_setenv progress
    path_swu_file=${path_swupdate}/*.swu
    tar -xzvf ${path_swu_file} -C ${target_path_tmp}
    if [ $? != 0 ];then
        echo -e "\nota_upgrade fail_untar_swu: untar failed."
        upgrade_setenv failed
        exit 1
    else
        mv ${target_path} ${target_path_bak}
        mv ${target_path_tmp} ${target_path}
        if [ $? != 0 ];then
            mv ${target_path_bak} ${target_path}
            echo -e "\nota_upgrade fail_remove: remove failed."
            upgrade_setenv failed
            exit 1
        else
            rm -rf ${target_path_bak}
            #todo backup xs target file.
            xs_nav_config_path=${xs_nav_target_path}/LogisticsConfig/UBT-G100/lidar
            xs_nav_config_bak_path=${xs_nav_target_path}/LogisticsConfig/UBT-G100/lidar.backup
            is_config_exist="no"
            if [ -d "${xs_nav_config_path}" ];then
                echo "\n LogisticsConfig alreay exit."
                is_config_exist="yes"
                echo "\n LogisticsConfig backup."
                mv ${xs_nav_config_path} ${xs_nav_config_bak_path}
            fi
            cd ${xs_nav_target_path}
            tar -xzvf ${xs_pack_file}
            if [ $? != 0 ];then
                echo -e "\nota_upgrade fail_untar_xs_nav_pack: untar xs nav pack failed."
                upgrade_setenv failed
                exit 1
            else
                if [ -d "${xs_nav_config_path}" ];then
                    echo "\n new LogisticsConfig been created."
                     if [ "${is_config_exist}" = "yes" ];then
                        echo "\n Delete new LogisticsConfig."
                        rm -rf ${xs_nav_config_path}
                        echo "\n Restore LogisticsConfig."
                        mv ${xs_nav_config_bak_path} ${xs_nav_config_path}
                    fi
                fi
                xs_nav_bin_pack_file=${xs_nav_target_path}/logisticsbinary_ubuntu18_x86.tar.gz

                if [ -f "${xs_nav_bin_pack_file}" ];then
                    echo -e "\n xs nav bin pack exists"
                    tar -xzvf ${xs_nav_bin_pack_file}
                else
                    echo -e "\nota_upgrade fail_untar_xs_nav_bin_pack: xs nav bin pack do not exist."
                    exit 1
                fi
                
                if [ $? != 0 ];then
                    echo -e "\nota_upgrade fail_untar_xs_nav_bin_pack: untar xs nav bin pack failed."
                    upgrade_setenv failed
                    exit 1
                else
                    rm -rf ${xs_nav_bin_pack_file}
                    upgrade_setenv success
                    echo -e "\nota_upgrade success: pepare restart now!"
                    exit 0
                fi
            fi
        fi
    fi
}

upgrade_pepare() {
    if [ ! -e ${path_tar_file} ]; then
        echo -e "\nota_pepare fail_no_found_tar: no found path_tar_file - ${path_tar_file}"
        exit 1
    fi

    # utar path_tar_file
    rm -rf ${path_swupdate_temp}/*
    tar -xzf ${path_tar_file} -C ${path_swupdate_temp}
    if [ $? != 0 ];then
        echo -e "\nota_pepare fail_atris_pack_untar: untar ${path_tar_file} failed."
        exit 1
    fi

    path_swu_file=${path_swupdate_temp}/*.swu
    path_swu_md5_file=${path_swupdate_temp}/*.swu.md5
    echo -e "\nota_pepare: path_swu_file ==> ${path_swu_file}"
    echo -e "\nota_pepare: path_swu_md5_file ==> ${path_swu_md5_file}"
    if [ ! -e ${path_swu_file} ]; then
        echo -e "\nota_pepare fail_no_found_swu: no found path_swu_file - ${path_swu_file}"
        exit 1
    fi
    if [ ! -e ${path_swu_md5_file} ]; then
        echo -e "\nota_pepare fail_no_found_swu_md5: no found swu_md5_file - ${path_swu_md5_file}"
        exit 1
    fi

    swu_file_md5=`md5sum ${path_swu_file} | gawk '{print $1}' | tr '[a-z]' '[A-Z]'`
    swu_md5_file_md5=`cat ${path_swu_md5_file} | gawk '{print $1}' | tr '[a-z]' '[A-Z]'`
    echo -e "\nota_pepare: input md5 ==> ${swu_md5_file_md5}."
    echo -e "\nota_pepare: check md5 ==> ${swu_file_md5}."
    if [ ${swu_file_md5} != ${swu_md5_file_md5} ]; then
        echo -e "\nota_pepare fail_check_md5: check md5 failed."
        exit 1
    fi


    # delete old swu files
    rm -rf ${path_swupdate}/*.swu
    rm -rf ${path_swupdate}/*.swu.md5
    mv ${path_swupdate_temp}/* ${path_swupdate}
    if [ $? != 0 ];then
        echo -e "\nota_pepare fail_unknow: unknow."
        exit 1
    else
        upgrade_progress
    fi
}

recovery_status=`upgrade_printenv`
if [ -z ${path_tar_file} ] ;then
    if [ "${recovery_status}" == "progress" ] ;then
        upgrade_progress
    fi
else
    upgrade_pepare
fi


