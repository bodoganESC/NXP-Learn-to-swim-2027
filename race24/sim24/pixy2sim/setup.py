from setuptools import setup, Extension
# enable compiler dependent optimizations >> https://stackoverflow.com/questions/6928110/how-may-i-override-the-compiler-gcc-flags-that-setup-py-uses-by-default
from setuptools.command.build_ext import build_ext
from pybind11 import get_include

pixy2sim_module = Extension('pixy2sim', sources =  ['pixy.cpp', 'fw.cpp', 'misc.cpp']
		,include_dirs = [get_include(),]
        #,extra_compile_args=["-Ofast"]
        )

class build_ext_ex(build_ext):
    extra_compile_args = {
        'pixy2sim': {
            'unix': ['-Ofast'],
            'mingw32': ['-Ofast'],
            'msvc': ['/O2'],
            'bcpp':[],
            'cygwin':[],
        }
    }

    def build_extension(self, ext):
        extra_args = self.extra_compile_args.get(ext.name)
        if extra_args is not None:
            ctype = self.compiler.compiler_type
            ext.extra_compile_args = extra_args.get(ctype, [])

        build_ext.build_extension(self, ext)
setup(
        name='pixy2sim',
        version='1.0',
        description='Accurately simulate Pixy2 processing using adapted parts of its firmware source',
        ext_modules=[pixy2sim_module],
        cmdclass= {'build_ext':build_ext_ex}
        )
