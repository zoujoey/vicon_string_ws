from setuptools import setup

package_name = 'string_tf_saver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Save string11 and string12 tf positions to a YAML file.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_string_positions = string_tf_saver.save_string_positions:main',
        ],
    },
)