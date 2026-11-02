from setuptools import setup
package_name = 'object_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=['object_detection', 'object_detection.deliver', 'object_detection.amz', 'object_detection.etc', 'object_detection.traffic'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/object_detection.launch.xml']),
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
            'yolo_deli = object_detection.deliver.yolo_deli:main',
            'yolo_amz = object_detection.amz.yolo_amz:main',
            'yolo_amz_sub = object_detection.amz.yolo_amz_sub:main',
            'yolo_amz_sub_sync = object_detection.amz.yolo_amz_sub_sync:main',
            'sub_image = object_detection.etc.sub_image:main',
            'yolo_traffic_flir = object_detection.traffic.yolo_traffic_flir:main',
        ],
    },
)