#!/usr/bin/env python

"""
setup.py file for SWIG libfdt
"""

from distutils.core import setup, Extension
import os
import sys

progname = sys.argv[0]
cflags = sys.argv[1]
files = sys.argv[2:]

if cflags:
    cflags = [flag for flag in cflags.split(' ') if flag]
else:
    cflags = None

libfdt_module = Extension(
    '_libfdt',
    sources = files,
    extra_compile_args =  cflags
)

sys.argv = [progname, '--quiet', 'build_ext', '--inplace']

setup (name = 'libfdt',
       version = '0.1',
       author      = "Simon Glass <sjg@chromium.org>",
       description = """Python binding for libfdt""",
       ext_modules = [libfdt_module],
       py_modules = ["libfdt"],
       )
