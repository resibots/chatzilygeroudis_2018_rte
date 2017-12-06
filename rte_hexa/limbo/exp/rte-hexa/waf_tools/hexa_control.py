#! /usr/bin/env python
# encoding: utf-8
# Konstantinos Chatzilygeroudis - 2015

"""
Quick n dirty hexa_control detection
"""

import os, glob, types
from waflib.Configure import conf
import re


def options(opt):
	opt.add_option('--hexa_control', type='string', help='path to hexa_control', dest='hexa_control')

@conf
def check_hexa_control(conf):
	if conf.options.hexa_control:
		conf.env.INCLUDES_HEXACONTROL = [conf.options.hexa_control + '/include']
	else:
		if 'ROS_PACKAGE_PATH' not in os.environ:
			conf.start_msg('Checking for hexa_control')
			conf.end_msg('Not found', 'RED')
			return
		path = os.environ['ROS_PACKAGE_PATH']
		paths = re.split(":",path)
		path = os.path.join(paths[0], 'hexapod_ros/hexapod_driver/include')
		conf.env.INCLUDES_HEXACONTROL = [path]
		conf.env.LIBPATH_HEXACONTROL = [os.path.join(paths[0], '../devel/lib')]
		conf.env.LIB_HEXACONTROL = ['hexapod_control']

	try:
		conf.start_msg('Checking for hexa_control')
		res = conf.find_file('hexapod_driver/hexapod.hpp', conf.env.INCLUDES_HEXACONTROL)
		res = res and conf.find_file('libhexapod_control.so', conf.env.LIBPATH_HEXACONTROL)
		conf.end_msg('ok')
	except:
		conf.end_msg('Not found', 'RED')
		return
	return 1
