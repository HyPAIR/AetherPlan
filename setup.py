from setuptools import setup, find_packages

setup(
    name="pddlstream",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "pybullet",
        "scipy"
    ],
    python_requires=">=3.6",  # Minimum Python version required
    author="saleeq: Adopted from caelan",
    author_email="saleeqmohammed7@gmail.com",
    description="installable verison of pddlstream",
    url="https://github.com/caelan/pddlstream/tree/main",  
)