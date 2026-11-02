from setuptools import setup

package_name = 'traffic_yolo'

setup(
    name=package_name,
    version='2.0.0',
    packages=['traffic_yolo'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/traffic_yolo.launch.xml']),
        ('lib/' + package_name + '/models/',['models/experimental.py', 'models/common.py',
                                             'models/yolo.py']),
        ('lib/' + package_name + '/utils/',['utils/general.py', 'utils/torch_utils.py','utils/visualizer.py',
                                             'utils/plots.py', 'utils/dataloaders.py',
                                             'utils/activations.py', 'utils/downloads.py',
                                             'utils/augmentations.py', 'utils/lion.py',
                                             'utils/autoanchor.py','utils/__init__.py',
                                             'utils/loss.py', 'utils/metrics.py']),
        ('lib/' + package_name + '/utils/segment/',['utils/segment/general.py']),
        ('lib/' + package_name + '/utils/tal/',['utils/tal/__init__.py','utils/tal/anchor_generator.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JooHoPark',
    maintainer_email='anAlyazzooom@secret.kr',
    description='A ROS 2 package to deploy YOLOv9 trained models on',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_yolo = traffic_yolo.traffic_yolo:main'
        ],
    },
)
