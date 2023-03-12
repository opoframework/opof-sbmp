import multiprocessing
import os
import pathlib
import sysconfig

from setuptools import Extension, find_namespace_packages, setup
from setuptools.command.build_ext import build_ext as build_ext_orig

with open(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "README.md"),
    encoding="utf-8",
) as f:
    long_description = f.read()


def package_files(root_dir, directory):
    paths = []
    for (path, _, filenames) in os.walk(root_dir + "/" + directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename)[len(root_dir) + 1 :])
    return paths


class CMakeExtension(Extension):
    def __init__(self, name):
        # don't invoke the original build_ext for this special extension
        super().__init__(name, sources=[])


class build_ext(build_ext_orig):
    def run(self):
        for ext in self.extensions:
            self.build_cmake(ext)
        super().run()

    def build_cmake(self, ext):
        cwd = pathlib.Path().absolute()

        # these dirs will be created in build_py, so if you don't have
        # any python sources to bundle, the dirs will be missing
        build_temp = pathlib.Path(self.build_temp)
        build_temp.mkdir(parents=True, exist_ok=True)
        extdir = pathlib.Path(self.get_ext_fullpath(ext.name))
        extdir.mkdir(parents=True, exist_ok=True)

        # example of cmake args
        config = "Debug" if self.debug else "Release"
        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=" + str(extdir.parent.absolute()),
            "-DCMAKE_BUILD_TYPE=" + config,
            "-DPYTHON_INCLUDE_DIRS=" + sysconfig.get_path("include"),
        ]

        # example of build args
        build_args = ["--config", config, "--", f"-j{multiprocessing.cpu_count()}"]

        os.chdir(str(build_temp))
        self.spawn(["cmake", str(cwd)] + cmake_args)
        self.spawn(["cmake", "--build", "."] + build_args)
        # Troubleshooting: if fail on line above then delete all possible
        # temporary CMake files including "CMakeCache.txt" in top level dir.
        os.chdir(str(cwd))


setup(
    name="opof-sbmp",
    version="0.2.0",
    description="OPOF domains for sampling-based robot motion planning",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Yiyuan Lee",
    author_email="yiyuan.lee@rice.edu",
    project_urls={
        "Source": "https://github.com/opoframework/opof-sbmp",
    },
    url="https://github.com/opoframework/opof-sbmp",
    license="BSD-3",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Mathematics",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development",
        "Topic :: Software Development :: Libraries",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "License :: OSI Approved :: BSD License",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    python_requires=">=3.9",
    install_requires=[
        "opof",
        "torch",
        "pybullet",
        "tqdm",
        "pyyaml",
        "urdf_parser_py",
        "numpy",
    ],
    ext_modules=[CMakeExtension("smbp_core")],
    cmdclass={
        "build_ext": build_ext,
    },
    extras_require={"tests": ["pytest", "pytest-cov", "pytest-timeout"]},
    packages=find_namespace_packages(),
    include_package_data=True,
    keywords="opof, optimization, machine learning, reinforcement learning, planning, motion planning, robotics",
)
