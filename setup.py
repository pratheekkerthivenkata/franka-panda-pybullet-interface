from os import path

import setuptools

this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, 'README.md')) as f:
    long_description = f.read()

setuptools.setup(
    name='franka_panda_pybullet_interface',
    author="Anuj Pasricha",
    author_email='anuj.pasricha@colorado.edu',
    version='1.0.0',
    description='A simulation interface for the Franka Emika Panda robot.',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/a-nooj/franka-panda-pybullet-interface',
    packages=setuptools.find_packages(where='franka_panda_pybullet_interface*'),
    install_requires=['numpy', 'pybullet==3.2.5', 'pyudev==0.24.1', 'numbalsoda==0.3.4'],
    classifiers=[
        'Programming Language :: Python :: 3.8',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.8'
)
