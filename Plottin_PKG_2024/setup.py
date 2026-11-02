from setuptools import setup

package_name = 'plotting_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'plotting_pkg.plotting_node'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ulix7990',
    maintainer_email='ulix7990@gmail.com',
    description='A package to plot paths and trajectories',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plotting_node = plotting_pkg.plotting_node:main'
        ],
    },
)
