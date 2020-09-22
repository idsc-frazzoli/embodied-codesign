from setuptools import setup

setup(
    name='embodied-codesign',
    version='1.0',
    packages=['src', 'src.controller', 'src.sensing', 'src.simulator', 'src.vehicle'],
    url='https://github.com/idsc-frazzoli/embodied-codesign',
    license='',
    author='Dejan Milojevic',
    author_email='dejan.milojevic@empa.ch',
    description='Simulator for embodied co-design.'
)