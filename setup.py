from setuptools import setup, find_packages

setup(name='safedrones',
      version='0.0.1',
      description='A Framework for Reliability/Safety Modelling and Evaluation of Multicopters (Multi-rotor Drones) and Electric powered Vertical TakeOff and Landing (eVTOL) Aircrafts.',
      url='https://github.com/Dependable-Intelligent-Systems-Lab/SafeDrones',
      author='Koorosh Aslansefat (corresponding), Panagiota Nikolaou, Mohammad Naveed Akram, Ioannis Sorokos, Martin Walker, Yiannis Papadopoulos, et al.',
      author_email='koo.ec2008@gmail.com',
      license='BSD',
      packages=find_packages(exclude=['js', 'node_modules', 'tests']),
      python_requires='>=3.5',
      install_requires=[
          'matplotlib',
          'numpy',
          'scipy',
          'tqdm >= 4.29.1',
          'scikit-learn>=0.18',
          'scikit-image>=0.12',
          'pyDOE2==1.3.0',
          'twine==1.13.0'
      ],
      extras_require={
          'dev': ['pytest', 'flake8'],
      },
      include_package_data=True,
      zip_safe=False)
