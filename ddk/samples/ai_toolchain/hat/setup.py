import io
import re

from setuptools import find_packages, setup

import hdlt

license_str = open("LICENSE").read()

version = hdlt.libinfo.get_version(hdlt.__version__)

setup(
    name="hdlt",
    version=version,
    author="HDLT Contributors",
    description="Horizon Deep Learning Toolkit",
    license=license_str,
    packages=find_packages(),
    install_requires=requirements,
)
