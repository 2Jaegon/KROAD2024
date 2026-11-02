from setuptools import setup

package_name = 'local_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ('share/' + package_name + '/launch', ['launch/local.launch.py']),
         ('share/' + package_name + '/launch', ['launch/real_heading.launch.py']),
         ('share/' + package_name + '/launch', ['launch/amz.launch.py']),
         ('share/' + package_name + '/launch', ['launch/amztest.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                  'morai_gps = local_pkg.morai_gps:main',
                  'morai_heading = local_pkg.morai_imu:main',
                  'utm = local_pkg.gps_to_utm:main',
                  'map = local_pkg.map_recorder:main',
                  'gps_heading = local_pkg.gps_heading:main',
                  'gps_plot = local_pkg.gps_plot:main',
                  'gps_heading_plot = local_pkg.heading_plot:main',
                  'tae_localization = local_pkg.tae_localization:main',
                  'tae_heading = local_pkg.tae_heading:main',
                  'tae_utm = local_pkg.tae_utm:main',
                  'path_smoothing = local_pkg.path_smoothing:main',
                  'real_heading = local_pkg.real_heading_by_smc2000:main',
                  'curvature_map= local_pkg.curve_ver2:main',
        ],
    },
)
