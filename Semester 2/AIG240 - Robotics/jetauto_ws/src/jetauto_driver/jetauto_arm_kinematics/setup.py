import os
import sys
from distutils.core import setup

print('sudo python3 setup.py install')

setup(name="kinematics",
      version="1.3", 
      description="kinematics for arm", 
      author="aiden", 
      packages=['kinematics'],
      package_dir={'kinematics':'./src'}, )

if sys.argv[1] == 'install':
    try:
        os.system('sudo cp ./src/kinematics.so /usr/local/lib/python' + str(sys.version[:3]) + '/dist-packages/kinematics/')
    except BaseException as e:
        print(e)
