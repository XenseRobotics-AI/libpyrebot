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
            # Include each model's URDF, offset, meshes and source/license metadata.
            *[
                str(p.relative_to(Path(__file__).parent / "rebot_rt"))
                for p in (Path(__file__).parent / "rebot_rt/assets").rglob("*")
                if p.is_file()
            ],
        ]
    },
)
