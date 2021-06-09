#!/usr/bin/python
# -*- coding:utf-8 -*-

# 将一个文件转换成C语言数组
import sys
import numpy
import binascii

import os
enter_list = "\r"
next_p = "file_NULL"
def toc(file,outf):
	global enter_list
	global next_p
	textFile = open(file, 'rb') # 打开文件
	htmlName = os.path.basename(file) #sys.argv[1]
	arrayName = htmlName.replace('.','_');
	arrayName = arrayName.replace('-','_');

	outf.write("static const unsigned char data_" + arrayName + "[] = {"  + "\r")
	str_line = "    " #空出4个空格
	htmlName = "/" + htmlName
	htmlLen = len(htmlName) + 1
	for i in htmlName:		
		str_line += "0x{0:02x}".format(ord (i))+ ", "
	outf.write("    /* " + htmlName + "  (" + str(htmlLen) + " chars) */\r")	
	outf.write(str_line + "0x00, \r")
	
	while True:
		data = textFile.read(16) # 每一次读取16个字符
		if not data:
			break
		str_line = "    " #空出4个空格
		for i in data:		
			str_line += "0x{0:02x}".format(i)+ ", "
		outf.write(str_line + "\r")
	outf.write("};"  + "\r\n")
	
	enter_list += "const struct fsdata_file file__" + arrayName  + "[] = { {\r"
	enter_list += next_p  + ",\r"
	enter_list += "data_" + arrayName  + ",\r"
	enter_list += "data_" + arrayName + " + " + str(htmlLen)  + ",\r"
	enter_list += "sizeof(data_" + arrayName + ") - " + str(htmlLen)  + ",\r"
	enter_list += "1,"  + "\r"
	enter_list += "}};"  + "\r\r\r"
	next_p = "file__" + arrayName


# 遍历文件夹
def walkFile(file,outf):

	
    for root, dirs, files in os.walk(file):

        # root 表示当前正在访问的文件夹路径
        # dirs 表示该文件夹下的子目录名list
        # files 表示该文件夹下的文件list

        # 遍历文件

        for f in files:
            toc(os.path.join(root, f), outf)
            print(os.path.join(root, f))


#walkFile(os.path.dirname(sys.argv[1]))
#(filepath, htmlName) = os.path.split(sys.argv[1])
#htmlName = os.path.basename(sys.argv[1]) #sys.argv[1]
#arrayName = htmlName.replace('.','_');

f_output = open(sys.argv[2], "w")

#add head 
f_output.write("#include \"fs.h\"" + "\r")
f_output.write("#include \"lwip/def.h\"" + "\r")
f_output.write("#include \"fsdata.h\"" + "\r\r")
f_output.write("#define file_NULL (struct fsdata_file *) NULL" + "\r\r\r")

walkFile(sys.argv[1],f_output) 


f_output.write(enter_list)

f_output.write("#define FS_ROOT "  + next_p + "\r\n")
f_output.write("#define FS_NUMFILES 1"  + "\r\n")

# python html2hex.py hello_world.html hello_world