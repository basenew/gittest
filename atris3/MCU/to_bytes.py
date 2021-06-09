##########################################################################
# File Name: to_bytes.py
# Author: drew
# mail: xiaoshu.xie@ubtrobot.com
# Created Time: Thu 03 Sep 2020 02:40:26 PM CST
#########################################################################
#! /usr/bin/env python
# -*- coding:utf-8 -*-
import zlib

def to_bytes(n, length, endianess='big'):
    h = '%x' % n
    s = ('0'*(len(h) % 2) + h).zfill(length*2).decode('hex')
    return s if endianess == 'big' else s[::-1]


def revert(s, unit=2):
    res=""
    for i in range(len(s), unit-1, -unit):
        res = res + s[i-unit:i]
    return res

if __name__=="__main__":
    res = zlib.crc32('1225555555') & 0xffffffff
    print(to_bytes(res, 4))
    #print("0x0012345678")
    #print(revert("0x0012345678"))
