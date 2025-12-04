from setuptools import find_packages, setup

package_name = 'rbpf_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seifelshafey',
    maintainer_email='seifelshafey@todo.todo',
    description='Rao-Blackwellized Particle Filter SLAM',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'run_slam = rbpf_slam.fast_slam_1:main',
            'run_slam2 = rbpf_slam.fast_slam_2:main'
        ],
    },
)