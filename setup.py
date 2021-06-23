import os

from setuptools import setup, find_packages

dir_path = os.path.dirname(os.path.realpath(__file__))

with open(os.path.join(dir_path, 'src', 'sfers', 'version.py')) as fp:
    exec(fp.read())


def read_requirements_file(filename):
    req_file_path = '%s/%s' % (dir_path, filename)
    with open(req_file_path) as f:
        return [line.strip() for line in f]


packages = find_packages('src')
# Ensure that we don't pollute the global namespace.
for p in packages:
    assert p == 'sfers' or p.startswith('sfers.')


def pkg_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('../..', path, filename))
    return paths


setup(
    name='sfers',
    version=__version__,
    author='Zhiwu Zheng',
    url='https://github.com/zhiuwz/sfers.git',
    license='Zlib',
    packages=packages,
    package_dir={'': 'src'},
    python_requires='>=2.7.*, <=3.9.1',
    classifiers=[
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3.7",
        "License :: OSI Approved :: Zlib License",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Framework :: Robot Framework"
    ],
    install_requires=read_requirements_file('requirements.txt'),
)
