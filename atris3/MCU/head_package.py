#!/usr/bin/python
import os
import sys
import shutil
import struct
import analysis_common as sis
import hashlib
import numpy as np
import time,datetime
from zlib import crc32
#import ctypes
import to_bytes as tb

# import logging,time,os,sys,datetime,struct
# from Crypto.Cipher import AES
# from ctypes import *
# import fastlz
# import quicklz
# from ini_operation import ini_file_operator
#from log import config_logging

HEADER_TYPE = "RBL"

"""
argv[1] ubt_common.h 
argv[2] source bin file
argv[3] package out bin file
argv[4] app/bootloader flag (app bl)

"""

print(sys.argv[1])
sis.inputParameter(sys.argv[1])
read_file_name = sys.argv[2]
package_out_file_name = sys.argv[3]
app_bl = sys.argv[4]
hw_version = sis.getHardwareVersion()
compatible_hardware_version = sis.getCompatibleHwVersion()

if app_bl == "app":
    sw_version = sis.getAppSoftwareVersion()
elif app_bl == "bl":
    sw_version = sis.getBlSoftwareVersion()
else:
    print( 'target part error: '+ sw_version)
    exit

# compress_mode = sis.getCompress_mode()
# key_encrypt= sis.getKeyEncrypt()
# iv_encrypt = sis.getIvEncrypt()
# encryption_algorithm =sis.getEncryptionAlgorithm()



'''
typedef struct {
	char type[4];           
	rt_uint16_t fota_algo;   
	rt_uint8_t fm_time[6];	
	char target_part_name[16];
	char software_version[24];
	char hardware_version[24]; 
	rt_uint32_t code_crc; 
	rt_uint32_t reserved; 
	rt_uint32_t raw_size; 
	rt_uint32_t com_size;  
	rt_uint32_t head_crc; 
} rt_fota_part_head, *rt_fota_part_head_t;
'''

print("header infomation...")
print("app_bl:%#s" %app_bl)
print("sw_version:%#s" %sw_version)
print("hw_version:%#s" %hw_version)
print("compatible_hardware_version:%#s" %compatible_hardware_version)




rt_fota_part_head_type = np.dtype({
    'names':['type', 'fota_algo', 'fm_time','target_part_name','software_version','hardware_version','code_crc','reserved','raw_size','com_size'],
    'formats':['S4','<u2', 'S6','S16','S24','S24','<u4','<u4','<u4','<u4']})
rt_fota_head = np.array([(HEADER_TYPE,0,'112233','app','0.0.0','0.0.0',0,0,0,0)], dtype=rt_fota_part_head_type)
print("struct size:%d"%sys.getsizeof(rt_fota_head))#a[0]['fm_time']

'''
def encrypt(text, password,iv):
   # bs = AES.block_size
   # iv = Random.new().read(bs)
   try:
      cipher = AES.new(password, AES.MODE_CBC, iv)
      bs=AES.block_size
      pad = lambda s: s + (bs - len(s) % bs) * bytes([bs - len(s) % bs])
      data = cipher.encrypt(pad(text))
      return (data)
   except Exception as e:
      print("Exception Logged")

def test_decompress(dt,mode):
   pass
        # if mode == 'quicklz':
        #     ts=quicklz.decompress(dt)
        # else:
        #     ts = fastlz.decompress(dt)
        # with open("decompress_test.bin",'wb') as fd:
        #     fd.write(ts)
def test_deencrypt(text, password,iv):
   pass
        # cipher = AES.new(password, AES.MODE_CBC, iv)
        # file_data=cipher.decrypt(text)
        # unpad = lambda s: s[0:-(s[-1])]
        # file_data=unpad(file_data)
        # if self.comboBox.currentText() == 'quicklz':
        #     self.test_decompress(file_data, 'quicklz')
        # elif self.comboBox.currentText() == 'fastlz':
        #     self.test_decompress(file_data, 'fastlz')

'''



'''
def compress_pack(data):
   return data

'''
'''
   pack_data=bytes()
   pos=0
   while pos<len(data):
      pos_data=data[pos:pos+4096]
      pos=pos+len(pos_data)

      if compress_mode == 'quicklz':
         file_data = quicklz.compress(pos_data)
      elif compress_mode == 'fastlz':
         file_data = fastlz.compress(pos_data)[4:]
      else:
         return data
      pack_data=pack_data+len(file_data).to_bytes(4, byteorder="big")+file_data
      return pack_data
'''



if package_out_file_name == '':
    print( 'package_out_file_name error'+ package_out_file_name)
    exit


'''            
key = key_encrypt
iv = iv_encrypt

if len(key)!=32:
    print('key_encrypt length != 32')
    exit

if len(iv)!=16:
    print('iv_encrypt length != 16')
    exit
'''

with open(read_file_name,'rb') as rfd:
        bin_size=os.path.getsize(read_file_name)
        print('bin file size(Byte):'+str(bin_size))
        file_data=rfd.read(bin_size)

        rt_fota_head[0]['raw_size']=bin_size
        rt_fota_head[0]['fota_algo']=0
        rt_fota_head[0]['fm_time'] = datetime.datetime.now().strftime('%Y%m%d%H%M').decode('hex')
        rt_fota_head[0]['software_version'] = str(sw_version).replace("\"", "")
        rt_fota_head[0]['target_part_name']= app_bl
        rt_fota_head[0]['hardware_version'] = hw_version + ',' + compatible_hardware_version
 
        #print('farme:'+str(hex(crc32(rt_fota_head)&0xffffffff)))


        new_file = package_out_file_name
        #print('----------file check:'+str(hex(crc32(file_data)&0xffffffff)))
        
        '''
        if encryption_algorithm == 'AES256':
           #file_data=encrypt(file_data, key, iv)
           test_deencrypt(file_data,key, iv)

           key_encrypt = encrypt(bytes(key), key, iv)
           #file_data = str(file_data) + str(key_encrypt) + str(len(key_encrypt))
        '''

        rt_fota_head[0]['code_crc'] = crc32(file_data)&0xffffffff
        rt_fota_head[0]['com_size'] = len(file_data)
        print('head code crc:0x%08x' % rt_fota_head[0]['code_crc'])
        print('real code crc:'+str(hex(crc32(file_data)&0xffffffff)))

        with open(new_file,'wb') as wfd:
            wfd.write(rt_fota_head)
            header_crc = crc32(rt_fota_head) & 0xffffffff
            wfd.write(tb.to_bytes(header_crc, 4, 'little'))

            #header_crc = bytes.decode(str(hex(crc32(rt_fota_head)).replace("0x", "")))

            # header_crc = bytes.decode(str(hex(ctypes.c_uint32(crc32(rt_fota_head)).value).replace("0x", "")))

            #header_crc = str(hex(crc32(rt_fota_head)&0xffffffff).replace("0x", ""))


            # header_crc = str(hex(crc32(rt_fota_head)&0xffffffff).replace("0x", "")[::-2]).decode('hex')
            
            #print('header_crc:'+str(hex(header_crc))

            #header_crc = crc32(rt_fota_head)&0xffffffff
            #print('header crc:0x%08x' % header_crc)
            #print("header_crc:{}".format(header_crc))
            #wfd.write(hex(header_crc).replace("0x", ""))
            
	        #wfd.write( hex(crc32(rt_fota_head)&0xffffffff).replace("0x", "") ) #head_crc
	        #wfd.write(str(hex( struct.pack('<I', crc32(rt_fota_head)&0xffff ) ).replace("0x", ""))) #head_crc

	        #wfd.write(str(  hex(crc32(rt_fota_head)&0xffff).replace("0x", "")  )) #head_crc

	        #wfd.write((crc32(rt_fota_head)&0xffffffff).to_bytes(4, byteorder="little"))
            wfd.write(file_data)
            print("trans over")

        print('package size(Byte):' + str(os.path.getsize(new_file)))
        print("%s package complete -->> %s"%(datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),new_file))
        print('ok','package complete')

