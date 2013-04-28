#! /usr/bin/env python
# encoding: utf-8

import sys, os, shutil, subprocess

from waflib import extras
from waflib.Context import Context
from waflib.Build import BuildContext

top = '.'
out = 'wbuild'

APPNAME='woggle'

def options(opt):
    opt.load('compiler_c asm')

def configure(conf):
    conf.load('compiler_c asm')

    # Load cross compiler tools
    if conf.find_program('arm-none-eabi-gcc'):
        conf.env.CC = 'arm-none-eabi-gcc'
        conf.env.LINK_CC = 'arm-none-eabi-gcc'
    if conf.find_program('arm-none-eabi-ar'):
        conf.env.AR = 'arm-none-eabi-ar'

    conf.find_program('arm-none-eabi-objcopy', var='OBJ_CPY')
    conf.find_program('arm-none-eabi-size', var='ARM_SIZE')

    # Assembler specific conf
    if conf.find_program('arm-none-eabi-gcc'):
        conf.env.AS= 'arm-none-eabi-gcc'
    conf.env.AS_TGT_F = ['-c', '-o']
    conf.env.ASLNK_TGT_F = ['-o']

    # Linker specific conf
    conf.env['SHLIB_MARKER'] = ''
    conf.env['STLIB_MARKER'] = ''

    # Flags
    genflags = ['-std=c99', '-Wall', '-Werror', '-fasm', '-fdata-sections', '-ffunction-sections']
    archflags = ['-mcpu=cortex-m3', '-mthumb']
    optflags =  ['-g', '-Os', '-fmerge-all-constants', '-fsee']

    conf.env['CFLAGS'] =  genflags + archflags + optflags
    conf.env['ASFLAGS'] = archflags

    ldscript = conf.path.find_resource('stm32/stm32f10x_flash_md.ld')
    conf.env['LINKFLAGS'] = ['-T%s' % ldscript.abspath(),
                             '-Wl,-Map=%s.map' % APPNAME,
                             '-Wl,--gc-sections'] + archflags

    # Defines
    conf.env['DEFINES'] = ['GCC_ARMCM3', 'STM32F10X_MD']



def build(bld):
    # STM32 DIR
    stm32_dir = bld.path.find_dir('stm32/STM32_USB-FS-Device_Lib_V3.1.0/Libraries')
    stm32_core_dir = stm32_dir.find_dir('CMSIS/Core/CM3')
    stm32_startup_dir = stm32_core_dir.find_dir('startup/gcc')
    stm32_stddriver_dir = stm32_dir.find_dir('STM32F10x_StdPeriph_Driver')
    stm32_stddriver_srcdir = stm32_stddriver_dir.find_dir('src')
    stm32_stddriver_incdir = stm32_stddriver_dir.find_dir('inc')
    stm32_usb_dir = stm32_dir.find_dir('STM32_USB-FS-Device_Driver')
    stm32_usb_srcdir = stm32_usb_dir.find_dir('src')
    stm32_usb_incdir = stm32_usb_dir.find_dir('inc')
    # FreeRTOS dir
    freertos_dir = bld.path.find_dir('freertos/Source')
    freertos_incdir = freertos_dir.find_dir('include')
    freertos_platdir = freertos_dir.find_dir('portable/GCC/ARM_CM3')
    freertos_memdir = freertos_dir.find_dir('portable/MemMang')
    # project dir
    lib_dir = bld.path.find_dir('lib')
    libglobal_dir = lib_dir.find_dir('libglobal')
    libperiph_dir = lib_dir.find_dir('libperiph')
    soft_dir = bld.path.find_dir('soft')

    # Build libstm32
    bld(features   = 'c cstlib',
        source     = stm32_stddriver_srcdir.ant_glob(['stm32f10x_flash.c',
                                                      'stm32f10x_gpio.c',
                                                      'stm32f10x_rcc.c',
                                                      'misc.c',
                                                      ]),
        target     = 'stm32',
        cflags     = ['-include', 'assert_param.h'],
        includes   = [stm32_stddriver_incdir.abspath(),
                      stm32_core_dir.abspath(),
                      libglobal_dir.abspath()],
        )

    # Build libperiph
    bld(features   = 'c cstlib',
        target     = 'periph',
        source     = libperiph_dir.ant_glob(['*.c']),
        includes   = [stm32_stddriver_incdir.abspath(),
                      stm32_core_dir.abspath()],
        )

    project_sources = []
    project_sources.extend(stm32_startup_dir.ant_glob(['startup_stm32f10x_md.s']))
    project_sources.extend(soft_dir.ant_glob(['*.c']))
    project_sources.extend(freertos_dir.ant_glob(['port.c', 'queue.c', 'tasks.c', 'list.c']))
    project_sources.extend(freertos_memdir.ant_glob(['heap_1.c']))
    project_sources.extend(freertos_platdir.ant_glob(['port.c']))

    # Build project
    bld(features   = 'asm c cprogram',
        source     = project_sources,
        target     = '%s.elf' % APPNAME,
        use        = ['periph', 'stm32'],
        includes   = [stm32_stddriver_incdir.abspath(),
                      stm32_core_dir.abspath(),
                      freertos_incdir.abspath(),
                      lib_dir.abspath(),
                      ],
        defines    = ['GCC_ARMCM3', 'STM32F10X_MD'],
        )

    # Create flash image
    bld(rule='${OBJ_CPY} -O binary ${SRC} ${TGT}', source='%s.elf' % APPNAME, target='flash.bin')

    # Copy flash configuration
    bld(rule='cp ${SRC} ${TGT}', source='flash/flash.cfg', target='flash.cfg')

def upload(upl):
    # Kill previous openocd instances
    os.system("killall -q openocd")
    # Flash into Olimexino
    openocd_cmd = ['openocd']
    openocd_cmd += ['-s']
    openocd_cmd += ['%s' % upl.path.find_dir('./wbuild').abspath()]
    openocd_cmd += ['-f']
    openocd_cmd += ['flash.cfg']
    subprocess.call(openocd_cmd)

class Upload(BuildContext):
    cmd = 'upload'
    fun = 'upload'
