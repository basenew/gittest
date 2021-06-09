#!/usr/bin/python
import os
import sys
import shutil
import struct
import analysis_common as sis
import hashlib



"""
argv[1] ubt_common.h  
argv[2] project name
argv[3] rbl file path
argv[4] out file name
argv[5] target part (app bl)

"""

sis.inputParameter(sys.argv[1])
project_name = sys.argv[2]
rbl_file_path = sys.argv[3]
out_file_name = sys.argv[4]
app_bl = sys.argv[5]

hw_version = sis.getHardwareVersion()
compatible_hardware_version = sis.getCompatibleHwVersion()
app_version = str(sis.getAppSoftwareVersion()).replace("\"", "")
bl_version = str(sis.getBlSoftwareVersion()).replace("\"", "")



print("generate config file...")
print(sys.argv[1])

print("project_name:%#s" %project_name)
print("out_file_name:%#s" %out_file_name)
print("app_bl:%#s" %app_bl)

print("hw_version:%#s" %hw_version)
print("compatible_hardware_version:%#s" %compatible_hardware_version)
print("app_version:%#s" %app_version)
print("bl_version:%#s" %bl_version)




def get_md5(file_path):
    f = open(file_path, 'rb')  
    md5_obj = hashlib.md5()
    while True:
        d = f.read(4096)
        if not d:
            break
        md5_obj.update(d)
    hash_code = md5_obj.hexdigest()
    f.close()
    md5 = str(hash_code).upper()
    return md5
 

appbin_path = rbl_file_path + "/app.rbl"
blbin_path = rbl_file_path + "/bootloader.rbl"
out_file_path = rbl_file_path + "/" + out_file_name

product = " - product:" + project_name + "\n"+ "\n"

target_part = "   target_part:" + app_bl + "\n"
source_part = "   source_part:" + "download" + "\n" + "\n"

app_version = "   app_version:" + app_version + "\n"
bl_version = "   bl_version:" + bl_version + "\n" + "\n"

app_md5 = "   app_md5:" + get_md5(appbin_path) + "\n"
bl_md5 = "   bl_md5:" + get_md5(blbin_path) + "\n" + "\n"

forceUp = "   forceUp:" + "true" + "\n" 



with open(out_file_path,'wb') as wfd:
    wfd.write(product)
    wfd.write(target_part)
    wfd.write(source_part)
    wfd.write(app_version)
    wfd.write(bl_version)
    wfd.write(app_md5)
    wfd.write(bl_md5)
    wfd.write(forceUp)
    print("generate config file over")
