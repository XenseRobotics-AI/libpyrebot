"""Package the CMake-built extension for the active Python interpreter."""

import sysconfig
from pathlib import Path

from setuptools import Distribution, setup


class NativeDistribution(Distribution):
    def has_ext_modules(self):
        return True


suffix = sysconfig.get_config_var("EXT_SUFFIX")
if not suffix or not (Path(__file__).parent / "rebot_rt" / f"_native{suffix}").is_file():
    raise RuntimeError("Build rebot_rt/_native with CMake for this Python before packaging")

setup(
    distclass=NativeDistribution,
    package_data={
        "rebot_rt": [
            f"_native{suffix}",
            "assets/*/README.md",
            "assets/*/urdf/*",
            "assets/*/meshes/*/*",
        ]
    },
)
