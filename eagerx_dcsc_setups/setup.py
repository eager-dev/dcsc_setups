import setuptools

setuptools.setup(
    name='eagerx_dcsc_setups',
    version='0.0',
    description='Interface between EAGERx and the DCSC setups',
    url='https://github.com/eager-dev/dcsc_setups/tree/main/eagerx_dcsc_setups',
    author='Bas van der Heijden, Jelle Luijkx',
    author_email='d.s.vanderheijden@tudelft.nl',
    license='Apache2.0',
    package_dir={"": "src"},
    packages=setuptools.find_packages(where="src"),
    python_requires=">=3.6",
)
