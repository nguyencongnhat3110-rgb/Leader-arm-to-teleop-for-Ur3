import os
from setuptools import find_packages, setup

package_name = 'my_ur_teleop'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Dùng đường dẫn tương đối (relative path) theo đúng chuẩn bắt buộc của colcon
for root, dirs, files in os.walk('models'):
    install_target = os.path.join('share', package_name, root)
    file_list = [os.path.join(root, f) for f in files]
    if file_list:
        data_files.append((install_target, file_list))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vinh',
    maintainer_email='vinh@todo.todo',
    description='Mô phỏng UR3e và Camera D435 với MuJoCo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_sim = my_ur_teleop.mujoco_sim_node:main',
            'ur3_teleop = my_ur_teleop.ur3_subscriber:main',
            'feetech_ctrl = my_ur_teleop.feetech_publisher:main',
            'vision_motion_node = my_ur_teleop.vision_motion_node:main',
            'cv_detection_node = my_ur_teleop.cv_detection_node:main',
            'feetech_mujoco = my_ur_teleop.feetech_publisher_sim:main',
            'ur3_teleop_sim = my_ur_teleop.mujoco_subscriber_sim:main',
            'calib_matrix = my_ur_teleop.matrix_5x5_mujoco:main',

        ],
    },
)