"""
Quick n dirty libfastsim detection
"""

import os, glob, types
from waflib.Configure import conf


def options(opt):
    opt.add_option('--libfastsim', type='string', help='path to libfastsim', dest='libfastsim')


@conf
def check_libfastsim(conf):
    if conf.options.libfastsim:
        includes_check = [conf.options.libfastsim + '/include']
        libs_check = [conf.options.libfastsim + '/lib']
    else:
        includes_check = ['/usr/local/include', '/usr/include']
        libs_check = ['/usr/local/lib', '/usr/lib', '/usr/lib/x86_64-linux-gnu/']
        if 'RESIBOTS_DIR' in os.environ:
            includes_check = [os.environ['RESIBOTS_DIR'] + '/include'] + includes_check
            libs_check = [os.environ['RESIBOTS_DIR'] + '/lib'] + libs_check

    try:
        conf.start_msg('Checking for libfastsim includes')
        res = conf.find_file('libfastsim/fastsim.hpp', includes_check)
        conf.end_msg('ok')
    except:
        conf.end_msg('Not found', 'RED')
        return 1
    conf.start_msg('Checking for libfastsim libs')
    found = False
    for lib in ['libfastsim.a']:
        try:
            found = found or conf.find_file(lib, libs_check)
        except:
            continue
    if not found:
        conf.end_msg('Not found', 'RED')
        return 1
    else:
        conf.end_msg('ok')
        conf.env.INCLUDES_LIBFASTSIM = includes_check
        conf.env.STLIBPATH_LIBFASTSIM = libs_check
        conf.env.STLIB_LIBFASTSIM = ['fastsim']
    return 1
