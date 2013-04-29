#!/usr/bin/env python
# encoding: utf-8

import os, sys
from waflib import Configure, Options, Utils
from waflib.Tools import ccroot, ar, gcc
from waflib.Configure import conf

from wtools import arm_ar

@conf
def find_arm_gcc(conf):
	"""
	Find the program gcc, and if present, try to detect its version number
	"""
	cc = conf.find_program(['arm-none-eabi-gcc', 'arm-linux-gnueabi-gcc'], var='CC')
	cc = conf.cmd_to_list(cc)
	conf.get_cc_version(cc, gcc=True)
	conf.env.CC_NAME = 'arm-gcc'
	conf.env.CC      = cc

def configure(conf):
	"""
	Configuration for arm-linux-gnueabi-gcc
	"""
	conf.find_arm_gcc()
        conf.find_arm_ar()
	conf.gcc_common_flags()
	conf.gcc_modifier_platform()
	conf.cc_load_tools()
	conf.cc_add_flags()
	conf.link_add_flags()



