#!/usr/bin/env python
# encoding: utf-8
# Thomas Nagy, 2008-2010 (ita)

"Detect as/gas/gcc for compiling assembly files"

import waflib.Tools.asm # - leave this
from wtools import arm_ar

def configure(conf):
	"""
	Find the programs arm-linux-gnueabi-/as/gcc and set the variable *AS*
	"""
        conf.find_program(['arm-none-eabi-gcc', 'arm-none-linux-gnueabi-gcc'], var='AS')
	conf.env.AS_TGT_F = ['-c', '-o']
	conf.env.ASLNK_TGT_F = ['-o']
	conf.find_arm_ar()
	conf.load('asm')
