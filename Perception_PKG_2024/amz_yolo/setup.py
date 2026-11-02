from setuptools import setup
package_name = 'amz_yolo'

setup(
    name=package_name,
    version='2.0.0',
    packages=['amz_yolo'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/amz_yolo.launch.xml']),
        ('lib/' + package_name + '/models/', ['models/experimental.py', 'models/common.py', 'models/yolo.py']),
        ('lib/' + package_name + '/utils/', [
            'utils/general.py', 'utils/torch_utils.py', 'utils/visualizer.py',
            'utils/plots.py', 'utils/datasets.py', 'utils/google_utils.py',
            'utils/activations.py', 'utils/add_nms.py', 'utils/autoanchor.py',
            'utils/loss.py', 'utils/metrics.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marno Nel & rshima',
    maintainer_email='marthinusnel2023@u.northwestern.edu & rintarohshima2023@u.northwestern.edu',
    description='A ROS 2 package to deploy YOLOv7 trained models on',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amz_yolo = amz_yolo.amz_yolo:main',
            'sub_amz_yolo = amz_yolo.sub_amz_yolo:main',
            'sync_amz_yolo = amz_yolo.sync_amz_yolo:main'
        ],
    },
)