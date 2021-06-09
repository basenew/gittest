#!/usr/bin/python2.7

hardware_version = ""
app_software_version = ""
jumper_linker_addr = ""
jumper_linker_size = ""
bootloader_linker_addr = ""
bootloader_linker_size = ""
app_linker_addr = ""
app_linker_size = ""
compile_mode = ""

def getValue(str,key):
   tem = str.replace("#define","")
   tem = tem.replace(key,"")
   tem = tem.lstrip()
   right = tem.find("//")
   if right > 0:
     tem = tem[0:right]
   return tem.rstrip()


print("collect file ubt_common.h values...")
f = open('./ubt_common.h','r')
lines = f.readlines()
for lines in lines:
    if "DF_COMPILE_RELEASE" in compile_mode:
      if "#else" in lines:
	      break

    if "DF_HARDWARE_VERSION" in lines:
      hardware_version = getValue(lines,"DF_HARDWARE_VERSION")
      print("hardware_version: %s" % hardware_version)

    if "DF_APP_SOFTWARE_VERSION" in lines:
      app_software_version = getValue(lines,"DF_APP_SOFTWARE_VERSION")
      print("app_software_version: %s" % app_software_version)

    if "DF_JUMPER_LINKER_ADDR" in lines:
      jumper_linker_addr = getValue(lines,"DF_JUMPER_LINKER_ADDR")
      print("jumper_linker_addr: %s" % jumper_linker_addr)

    if "DF_JUMPER_LINKER_SIZE" in lines:
      jumper_linker_size = getValue(lines,"DF_JUMPER_LINKER_SIZE")
      print("jumper_linker_size: %s" % jumper_linker_size)

    if "DF_BOOTLOADER_LINKER_ADDR" in lines:
      bootloader_linker_addr = getValue(lines,"DF_BOOTLOADER_LINKER_ADDR")
      print("bootloader_linker_addr: %s" % bootloader_linker_addr)

    if "DF_BOOTLOADER_LINKER_SIZE" in lines:
      bootloader_linker_size = getValue(lines,"DF_BOOTLOADER_LINKER_SIZE")
      print("bootloader_linker_size: %s" % bootloader_linker_size)

    if "DF_APP_LINKER_ADDR" in lines:
      app_linker_addr = getValue(lines,"DF_APP_LINKER_ADDR")
      print("app_linker_addr: %s" % app_linker_addr)

    if "DF_APP_LINKER_SIZE" in lines:
      app_linker_size = getValue(lines,"DF_APP_LINKER_SIZE")
      print("app_linker_size: %s" % app_linker_size)

    if "DF_COMPILE_RELEASE" in lines:
      if "#ifdef" in lines:
        continue

      #print(lines)
      right = lines.find("//")
      tem = lines

      if right >= 0:
        tem = tem[0:right]
      #print(tem)

      right = tem.find("/*")
      if right >= 0:
        tem = tem[0:right]

      compile_mode = tem
      print(compile_mode)
f.close()

print("collect values finish.\n")

print("modify jumper ld file :stm32_flash.ld ...")
source = open('jumper/stm32_flash.ld.pre','r')
destination = open('jumper/stm32_flash.ld','w')
lines = source.readlines()
for line in lines:
   if "FLASH (rx)" in line:
     #print(line)
     tem = line.replace("0x08000000",jumper_linker_addr)
     tem = tem.replace("8K",jumper_linker_size)
     tem = tem.replace("**","*don't change this file,this value define in ubt_common.h.by preprocessor.py change*")
     print(tem)
     destination.write(tem)
   else:
     destination.write(line)

     
source.close()
destination.close()
print("end modify jumper ld file :stm32_flash.ld")


print("modify bootloader ld file :link.lds")
source = open('bootloader/board/linker_scripts/link.lds.pre','r')
destination = open('bootloader/board/linker_scripts/link.lds','w')
lines = source.readlines()
for line in lines:
   if "CODE (rx)" in line:
     #print(line)
     tem = line.replace("0x08008000",bootloader_linker_addr)
     tem = tem.replace("224k",bootloader_linker_size)
     tem = tem.replace("**","*don't change this file,this value define in ubt_common.h.by preprocessor.py change*")
     print(tem)
     destination.write(tem)
   else:
     destination.write(line)
    
source.close()
destination.close()
print("end modify bootloader ld file :link.lds")


print("modify bootloader ld file :link.sct")
source = open('bootloader/board/linker_scripts/link.sct.pre','r')
destination = open('bootloader/board/linker_scripts/link.sct','w')
lines = source.readlines()
for line in lines:
   if "LR_IROM1" in line:
     #print(line)
     tem = line.replace("0x08008000",bootloader_linker_addr)
     tem_size = bootloader_linker_size.replace("k"," * 1024")
     tem = tem.replace("224k",tem_size)
     tem = tem.replace(";",";don't change this file,this value define in ubt_common.h.by preprocessor.py change")
     print(tem)
     destination.write(tem)
   elif "ER_IROM1" in line:
     #print(line)
     tem = line.replace("0x08008000",bootloader_linker_addr)
     tem_size = bootloader_linker_size.replace("k"," * 1024")
     tem = tem.replace("224k",tem_size)
     tem = tem.replace(";",";don't change this file,this value define in ubt_common.h.by preprocessor.py change")
     print(tem)
     destination.write(tem)
   else:
     destination.write(line)
    
source.close()
destination.close()
print("end modify bootloader ld file :link.sct")


print("modify app ld file :link.lds")
source = open('APP/board/linker_scripts/link.lds.pre','r')
destination = open('APP/board/linker_scripts/link.lds','w')
lines = source.readlines()
for line in lines:
   if "CODE (rx)" in line:
     #print(line)
     tem = line.replace("0x08040000",app_linker_addr)
     tem = tem.replace("768k",app_linker_size)
     tem = tem.replace("**","*don't change this file,this value define in ubt_common.h.by preprocessor.py change*")
     destination.write(tem)
     print(tem)
   else:
     destination.write(line)
    
source.close()
destination.close()
print("end modify APP ld file :link.lds")


print("modify APP ld file :link.sct")
source = open('APP/board/linker_scripts/link.sct.pre','r')
destination = open('APP/board/linker_scripts/link.sct','w')
lines = source.readlines()
for line in lines:
   if "LR_IROM1" in line:
     #print(line)
     tem = line.replace("0x08040000",app_linker_addr)
     tem_size = app_linker_size.replace("k"," * 1024")
     tem = tem.replace("768k",tem_size)
     tem = tem.replace(";",";don't change this file,this value define in ubt_common.h.by preprocessor.py change")
     print(tem)
     destination.write(tem)
   elif "ER_IROM1" in line:
     #print(line)
     tem = line.replace("0x08040000",app_linker_addr)
     tem_size = app_linker_size.replace("k"," * 1024")
     tem = tem.replace("768k",tem_size)
     tem = tem.replace(";",";don't change this file,this value define in ubt_common.h.by preprocessor.py change")
     print(tem)
     destination.write(tem)
   else:
     destination.write(line)
    
source.close()
destination.close()
print("end modify APP ld file :link.sct")

