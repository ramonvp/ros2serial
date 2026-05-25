from setuptools import setup
import os
package_name = 'rosserial_stm32'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
                
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=package_files(data_files, ['ros_lib']),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ramon Viedma',
    author_email='ramon.viedma@hp.com',
    maintainer='Ramon Viedma',
    maintainer_email='ramon.viedma@hp.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='An attempt to port rosserial to ROS2',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'make_libraries = rosserial_stm32.make_libraries:main'
        ],
    },
)
