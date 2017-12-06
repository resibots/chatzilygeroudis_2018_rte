#! /usr/bin/env python
# encoding: utf-8
# Konstantinos Chatzilygeroudis - 2015

"""
Quick n dirty hexapod_dart detection
"""

import os
import boost
import eigen
import hexapod_controller
import dart
from waflib.Configure import conf


def options(opt):
  opt.load('boost')
  opt.load('eigen')
  opt.load('hexapod_controller')
  opt.load('dart')
  opt.add_option('--hexapod_dart', type='string', help='path to hexapod_dart', dest='hexapod_dart')

@conf
def check_hexapod_dart(conf):
    conf.load('boost')
    conf.load('eigen')
    conf.load('hexapod_controller')
    conf.load('dart')

    # In boost you can use the uselib_store option to change the variable the libs will be loaded
    boost_var = 'BOOST_DART'
    conf.check_boost(lib='regex system', min_version='1.46', uselib_store=boost_var)
    conf.check_eigen()
    conf.check_hexapod_controller()
    conf.check_dart()

    includes_check = ['/usr/local/include', '/usr/include']
    libs_check = ['/usr/local/lib', '/usr/lib']

    # You can customize where you want to check
    # e.g. here we search also in a folder defined by an environmental variable
    if 'RESIBOTS_DIR' in os.environ:
    	includes_check = [os.environ['RESIBOTS_DIR'] + '/include'] + includes_check
    	libs_check = [os.environ['RESIBOTS_DIR'] + '/lib'] + libs_check

    if conf.options.hexapod_dart:
    	includes_check = [conf.options.hexapod_dart + '/include']
    	libs_check = [conf.options.hexapod_dart + '/lib']

    try:
    	conf.start_msg('Checking for hexapod_dart includes')
    	res = conf.find_file('hexapod_dart/hexapod.hpp', includes_check)
    	res = res and conf.find_file('hexapod_dart/hexapod_control.hpp', includes_check)
    	res = res and conf.find_file('hexapod_dart/hexapod_dart_simu.hpp', includes_check)
    	res = res and conf.find_file('hexapod_dart/descriptors.hpp', includes_check)
    	res = res and conf.find_file('hexapod_dart/safety_measures.hpp', includes_check)
    	conf.end_msg('ok')
    	conf.env.INCLUDES_HEXAPOD_DART = includes_check
    except:
    	conf.end_msg('Not found', 'RED')
    	return
    return 1
