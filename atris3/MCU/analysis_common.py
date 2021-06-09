#!/usr/bin/python2.7

hardware_version = ""
compatible_hardware_version = ""
app_software_version = ""
bl_software_version = ""
jumper_linker_addr = ""
jumper_linker_size = ""
bootloader_linker_addr = ""
bootloader_linker_size = ""
app_linker_addr = ""
app_linker_size = ""
compile_mode = ""
compress_mode = ""
key_encrypt = ""
iv_encrypt  = ""
encryption_algorithm = ""

def getCompatibleHwVersion():
   global compatible_hardware_version
   print(compatible_hardware_version)
   return compatible_hardware_version

def getEncryptionAlgorithm():
   global encryption_algorithm
   #print(encryption_algorithm)
   return encryption_algorithm

def getIvEncrypt():
   global iv_encrypt
   #print(iv_encrypt)
   return iv_encrypt

def getKeyEncrypt():
   global key_encrypt
   #print(key_encrypt)
   return key_encrypt

def getCompress_mode():
   global compress_mode
   #print(compress_mode)
   return compress_mode

def getHardwareVersion():
   global hardware_version
   #print(hardware_version)
   return hardware_version

def getAppSoftwareVersion():
   global app_software_version
   #print(app_software_version)
   return app_software_version

def getBlSoftwareVersion():
   global bl_software_version
   #print(bl_software_version)
   return bl_software_version

def getJumperLinkerAddr():
   global jumper_linker_addr
   #print(jumper_linker_addr)
   return eval(jumper_linker_addr)

def getJumperLinkerSize():
   global jumper_linker_size
   return eval(jumper_linker_size)

def getBootLoaderAddr():
   global bootloader_linker_addr
   return eval(bootloader_linker_addr)

def getBootloaderSize():
   global bootloader_linker_size
   return eval(bootloader_linker_size)

def getAppLinkerAddr():
   global app_linker_addr
   return eval(app_linker_addr)

def getAppLinkerSize():
   global app_linker_size
   return eval(app_linker_size)

def getValue(str,key):
   tem = str.replace("#define","")
   tem = tem.replace(key,"")
   tem = tem.lstrip()
   right = tem.find("//")
   if right > 0:
     tem = tem[0:right]
   tem = tem.rstrip()
   #print(tem)
   if "DF_PACK" in key:
 	return tem
   return tem.replace("k"," * 1024")

def inputParameter(common_file_name):
   global hardware_version
   global compatible_hardware_version
   global app_software_version
   global bl_software_version
   global jumper_linker_addr
   global jumper_linker_size
   global bootloader_linker_addr
   global bootloader_linker_size
   global app_linker_addr
   global app_linker_size
   global compile_mode
   global compress_mode
   global key_encrypt
   global iv_encrypt
   global encryption_algorithm
   f = open(common_file_name,'r')
   lines = f.readlines()
   for lines in lines:
      if "DF_COMPILE_RELEASE" in compile_mode:
        if "#else" in lines:
	   break
      if "DF_HARDWARE_VERSION" in lines:
         hardware_version = getValue(lines,"DF_HARDWARE_VERSION")
      if "DF_COMPATIBLE_HARDWARE_VERSION" in lines:
         compatible_hardware_version = getValue(lines,"DF_COMPATIBLE_HARDWARE_VERSION")
      if "DF_APP_SOFTWARE_VERSION" in lines:
        app_software_version = getValue(lines,"DF_APP_SOFTWARE_VERSION")
      if "DF_BL_SOFTWARE_VERSION" in lines:
        bl_software_version = getValue(lines,"DF_BL_SOFTWARE_VERSION")
      if "DF_JUMPER_LINKER_ADDR" in lines:
        jumper_linker_addr = getValue(lines,"DF_JUMPER_LINKER_ADDR")
      if "DF_JUMPER_LINKER_SIZE" in lines:
        jumper_linker_size = getValue(lines,"DF_JUMPER_LINKER_SIZE")
      if "DF_BOOTLOADER_LINKER_ADDR" in lines:
        bootloader_linker_addr = getValue(lines,"DF_BOOTLOADER_LINKER_ADDR")
      if "DF_BOOTLOADER_LINKER_SIZE" in lines:
        bootloader_linker_size = getValue(lines,"DF_BOOTLOADER_LINKER_SIZE")
      if "DF_APP_LINKER_ADDR" in lines:
        app_linker_addr = getValue(lines,"DF_APP_LINKER_ADDR")
      if "DF_APP_LINKER_SIZE" in lines:
        app_linker_size = getValue(lines,"DF_APP_LINKER_SIZE")
      if "DF_PACK_COMPRESS_MODE" in lines:
        compress_mode = getValue(lines,"DF_PACK_COMPRESS_MODE")
      if "DF_PACK_ENCRYPTION_ALGORITHM" in lines:
        encryption_algorithm = getValue(lines,"DF_PACK_ENCRYPTION_ALGORITHM")
      if "DF_PACK_KEY_ENCRYPT" in lines:
        key_encrypt = getValue(lines,"DF_PACK_KEY_ENCRYPT")
      if "DF_PACK_IV_ENCRYPT" in lines:
        iv_encrypt = getValue(lines,"DF_PACK_IV_ENCRYPT")
      if "DF_COMPILE_RELEASE" in lines:
        if "#ifdef" in lines:
           continue
        print(lines)
        right = lines.find("//")
        tem = lines
        if right >= 0:
           tem = tem[0:right]
        print(tem)
        right = tem.find("/*")
        if right >= 0:
           tem = tem[0:right]
        compile_mode = tem
   f.close()

def getFromulaValue(str):
   return eval(str)


