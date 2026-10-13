"""
Auxiliary script for setting the Nvidia libraries in the RADI

Register all pip-installed nvidia .so files with ldconfig so onnxruntime-gpu
finds them regardless of whether pip used site-packages or dist-packages
"""

import sysconfig, pathlib

site = pathlib.Path(sysconfig.get_path("purelib"))
conf = "\n".join(str(d) for d in site.glob("nvidia/*/lib") if d.is_dir())
pathlib.Path("/etc/ld.so.conf.d/nvidia-pip.conf").write_text(conf + "\n")
