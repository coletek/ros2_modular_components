from setuptools import setup

package_name = 'py_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,
              "%s.sensors" % package_name,
              "%s.devices" % package_name,
              "%s.proxies" % package_name,
              "%s.controllers" % package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='COLETEK PTY LTD',
    maintainer_email='dev@coletek.org',
    description='coletek.org CT',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
         'console_scripts': [
             'sensor_interrupts = py_hardware.sensors.interrupts:main',
             'sensor_mcp3008 = py_hardware.sensors.mcp3008:main',
             'sensor_onewire = py_hardware.sensors.onewire:main',
             'sensor_serial = py_hardware.sensors.serial:main',
             'sensor_i2c = py_hardware.sensors.i2c:main',
             'sensor_v4l = py_hardware.sensors.v4l:main',
             'device_buzzers = py_hardware.devices.buzzers:main',
             'device_fans = py_hardware.devices.fans:main',
             'device_hub75 = py_hardware.devices.hub75:main',
             'device_lights = py_hardware.devices.lights:main',
             'device_motors = py_hardware.devices.motors:main',
             'device_max7301atl = py_hardware.devices.max7301atl:main',
             'proxy_v4l = py_hardware.proxies.v4l:main'
        ],
    },
)
