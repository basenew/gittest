#!/usr/bin/python2.7
import os
import sys
import shutil
import struct
import analysis_common as sis

bin_path = sys.argv[2]
jumper_path = bin_path + 'jumper.bin'
bootloader_path = bin_path +  'bootloader.bin'
app_path = bin_path +  'app.bin'

bin_result_path = bin_path +  'burn.bin'


print(bin_path)
print(jumper_path)
print(bootloader_path)
print(app_path)
print(bin_result_path)


print(sys.argv[1])
sis.inputParameter(sys.argv[1])
"""
print("getJumperLinkerAddr:%#x"%sis.getJumperLinkerAddr())
print("getJumperLinkerSize:%#x"%sis.getJumperLinkerSize())
print("getBootLoaderAddr:%#x"%sis.getBootLoaderAddr())
print("getBootloaderSize:%#x"%sis.getBootloaderSize())
print("getAppLinkerAddr:%#x"%sis.getAppLinkerAddr())
print("getAppLinkerSize:%#x"%sis.getAppLinkerSize())
"""

df_jumper_offset = sis.getJumperLinkerAddr() & (~0x8000000)
df_bootloader_offset = sis.getBootLoaderAddr() & (~0x8000000)
df_app_offset = sis.getAppLinkerAddr() & (~0x8000000)

df_jumper_size = sis.getJumperLinkerSize() & (~0x8000000)
df_bootloader_size = sis.getBootloaderSize() & (~0x8000000)
df_app_size = sis.getAppLinkerSize() & (~0x8000000)

print("df_jumper_offset:%#x"%df_jumper_offset)
print("df_bootloader_offset:%#x"%df_bootloader_offset)
print("df_app_offset:%#x"%df_app_offset)

print("df_jumper_size:%#x"%df_jumper_size)
print("df_bootloader_size:%#x"%df_bootloader_size)
print("df_app_size:%#x"%df_app_size)

if (df_bootloader_offset == 0) & (df_app_offset == 0) :
  print("debug mode don't merge")
  sys.exit()

shutil.copyfile(jumper_path,bin_result_path)

jumper_bin = open(jumper_path,'rb')
bootloader_bin = open(bootloader_path,'rb')
app_bin = open(app_path,'rb')
bin_merge = open(bin_result_path, 'ab')

jumper_size = os.path.getsize(jumper_path)
print("jumper_size:%#x"%jumper_size)
if jumper_size > df_jumper_size:
  print('jumper bin size error,please check ubt_common.h and app stm32_flash.ld')
  sys.exit()

bootloader_size = os.path.getsize(bootloader_path)
print("bootloader_size:%#x"%bootloader_size)
if bootloader_size > df_bootloader_size:
  print('bootloader bin size error,please check ubt_common.h and bootloader link.lds')
  sys.exit()

app_size = os.path.getsize(app_path)
print("app_size:%#x"%app_size)
if app_size > df_app_size:
  print('app bin size error,please check ubt_common.h and app link.lds')
  sys.exit()


final_size = df_app_offset + app_size
print("final_size:%#x"%final_size)


offset = os.path.getsize(bin_result_path)
value_default = struct.pack('B', 0xFF)
print("offset:%#x"%offset)
#print(value_default)


while offset < final_size:
    if offset == df_bootloader_offset:
        data = bootloader_bin.read()
        bin_merge.write(data)
        offset = bin_merge.tell()
    elif offset == df_app_offset:
        data = app_bin.read()
        bin_merge.write(data)
        offset = bin_merge.tell()
    else:
        bin_merge.write(value_default)
        offset = bin_merge.tell()

jumper_bin.close()
bootloader_bin.close()
app_bin.close()

bin_merge.close()
