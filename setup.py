from setuptools import setup, find_packages

setup(
  name = 'general_motion_retargeting',
  packages = find_packages(),
  package_data={
      "general_motion_retargeting": ["opensim/mappings/*.json"],
  },
  include_package_data=True,
  author="Yanjie Ze",
  author_email="lastyanjieze@gmail.com",
  description="General Motion Retargeting (GMR) for Humanoid Robots",
  long_description=open("README.md").read(),
  long_description_content_type="text/markdown",
  url="https://github.com/YanjieZe/GMR",
  license="MIT",
  version="0.2.0",
  install_requires=[
    "loop_rate_limiters",
    "mink",
    "mujoco",
    "numpy",
    "scipy",
    "qpsolvers[proxqp]",
    "rich",
    "tqdm",
    "opencv-python",
    "natsort",
    "psutil",
    "smplx @ git+https://github.com/vchoutas/smplx",
    # chumpy: install separately (PyPI 0.70 is broken; git needs --no-build-isolation):
    #   pip install "git+https://github.com/mattloper/chumpy.git" --no-build-isolation
    "protobuf",
    "redis[hiredis]",
    "imageio[ffmpeg]",
  ],
  python_requires='>=3.10',
)
