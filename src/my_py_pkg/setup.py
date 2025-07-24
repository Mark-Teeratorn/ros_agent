from glob import glob
from setuptools import find_packages, setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mark',
    maintainer_email='mark@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_py_pkg.my_node:main',
            'count = my_py_pkg.count:main',
            'walk = my_py_pkg.walking_turtle:main',
            'command = my_py_pkg.command:main',
            'ai_ros_agent = my_py_pkg.command:main',
            'ai = my_py_pkg.ai_ros_agent:main'
        ],
    },
)
