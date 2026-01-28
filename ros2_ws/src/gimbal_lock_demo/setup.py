from setuptools import setup

package_name = 'gimbal_lock_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
        ('share/' + package_name + '/urdf', ['urdf/gimbal.urdf']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/gimbal.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='M76150518',
    maintainer_email='your_email@example.com',
    description='Gimbal lock demonstration using Euler and Quaternion control with rqt + RViz.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joint_controller = gimbal_lock_demo.joint_controller:main',
        ],
        'rqt_gui_py.plugins': [
            'rqt_gimbal = gimbal_lock_demo.rqt_gimbal_plugin:RqtGimbalPlugin',
        ],
    },
)

