from setuptools import setup

package_name = 'multi_rtd'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='talbot330-red',
    maintainer_email='adamdai97@gmail.com',
    description='Multirobot RTD',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_planner = multi_rtd.simple_planner:main',
            'multi_planner = multi_rtd.multi_planner:main',
            'trajectory_sampler = multi_rtd.trajectory_sampler:main',
        ],
    },
)
