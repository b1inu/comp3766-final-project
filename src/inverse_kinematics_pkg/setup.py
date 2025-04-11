from setuptools import find_packages, setup

package_name = 'inverse_kinematics_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/view_puma.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/puma.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='binu',
    maintainer_email='mennadmuneeb@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "numerical_ik_node = inverse_kinematics_pkg.numerical_inverse_kinematics:main",
            "analytical_ik_node = inverse_kinematics_pkg.analytical_inverse_kinematics:main",
            "goal_pose_node = inverse_kinematics_pkg.goal_pose_node:main",
        ],
    },
)
