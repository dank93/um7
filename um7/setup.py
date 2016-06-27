from setuptools import setup

setup(name='um7',
      version='0.8',
      description='Classes to interface with CH Robotics UM7 IMU',
      url='https://pypi.python.org/pypi/um7/0.8',
      author='Daniel Kurek',
      author_email='dkurek93@gmail.com',
      license='MIT',
      packages=['um7'],
      install_requires=['pyserial', 'numpy'],
      zip_safe=False)
