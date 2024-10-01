import os
from pathlib import Path

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

class CMakeExtension(Extension):
    def __init__(self, name):
        super().__init__(name, sources=[])

class BuildExt(build_ext):
    def run(self):
        for ext in self.extensions:
            if isinstance(ext, CMakeExtension):
                self.build_cmake(ext)
        super().run()

    def build_cmake(self, ext):
        base_dir = Path(__file__).resolve().parent
        os.chdir(base_dir)
        build_temp = Path(self.build_temp)
        os.makedirs(build_temp, exist_ok=True)
        ext_dir = Path(self.get_ext_fullpath(ext.name)).absolute()
        os.makedirs(ext_dir.parent, exist_ok=True)

        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=" + str(ext_dir.parent),
            "-DCMAKE_BUILD_TYPE=Release"
        ]

        build_args = ["-j6"]

        os.chdir(build_temp)
        self.spawn(["cmake", f"{base_dir}/{ext.name}"] + cmake_args)
        if not self.dry_run:
            self.spawn(["cmake", "--build", "."] + build_args)
        os.chdir(base_dir)

setup(
    name='pysrt3d',
    ext_modules=[
        CMakeExtension("source")
    ],
    cmdclass={
        'build_ext': BuildExt
    }
)
