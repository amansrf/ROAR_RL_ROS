from setuptools import setup

package_name = 'roar_env'
agent_module = 'roar_env/agent_module'
carla_client_module = 'roar_env/carla_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, agent_module, carla_client_module],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roar',
    maintainer_email='amansaraf99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
