# SFERS: Simulation Framework for Electrostatic Robotic Sheet
SFERS is an *open-source* project for simulating planar piezoelectric soft robots, one-dimensional arrays of actuators based on pseudo-rigid body model employed with PyBullet.
## Installation
SFERS is compatible with Python 3.6 - 3.9. The easiest way to install SFERS is to with PIP. 
```bash
git clone https://github.com/zhiwuz/sfers.git
cd sfers
pip install -r requirements.txt
pip install -e .
```
Test the installed package:
```bash
pytest tests/.
```
## Tutorials
We have created several Jupyter notebooks and Python scripts to help users get started with SFERS. The Jupyter notebooks are available on Binder, allowing you to try out some of the tutorials without having to install SFERS.

## Citations
We request that publications that use SFERS cite the following papers:

Overall simulation environment:

Zhiwu Zheng, Prakhar Kumar, Yenan Chen, Hsin Cheng, Sigurd Wagner, Minjie Chen, Naveen Verma and James C. Sturm. <strong>Scalable Simulation and Demonstration of Jumping Piezoelectric 2-D Soft Robots</strong>. arXiv preprint, arXiv: [2202.13521](https://arxiv.org/abs/2202.13521)

```
@misc{https://doi.org/10.48550/arxiv.2202.13521,
    doi = {10.48550/ARXIV.2202.13521},
    url = {https://arxiv.org/abs/2202.13521},
    author = {Zheng, Zhiwu and Kumar, Prakhar and Chen, Yenan and Cheng, Hsin and Wagner, Sigurd and Chen, Minjie and Verma, Naveen and Sturm, James C.},
    keywords = {Robotics (cs.RO), Systems and Control (eess.SY), FOS: Computer and information sciences, FOS: Computer and information sciences, FOS: Electrical engineering, electronic engineering, information engineering, FOS: Electrical engineering, electronic engineering, information engineering},
    title = {Scalable Simulation and Demonstration of Jumping Piezoelectric 2-D Soft Robots},
    publisher = {arXiv},
    year = {2022},
    copyright = {arXiv.org perpetual, non-exclusive license}
}
```

For modeling of piezoelectric robots:

Zhiwu Zheng, Prakhar Kumar, Yenan Chen, Hsin Cheng, Sigurd Wagner, Minjie Chen, Naveen Verma and James C. Sturm. <strong>Piezoelectric Soft Robot Inchworm Motion by Controlling Ground Friction through Robot Shape</strong>. arXiv preprint, arXiv: [2111.00944](https://arxiv.org/abs/2111.00944)
```
@misc{https://doi.org/10.48550/arxiv.2111.00944,
    doi = {10.48550/ARXIV.2111.00944},
    url = {https://arxiv.org/abs/2111.00944},
    author = {Zheng, Zhiwu and Kumar, Prakhar and Chen, Yenan and Cheng, Hsin and Wagner, Sigurd and Chen, Minjie and Verma, Naveen and Sturm, James C.},
    keywords = {Robotics (cs.RO), Systems and Control (eess.SY), FOS: Computer and information sciences, FOS: Computer and information sciences, FOS: Electrical engineering, electronic engineering, information engineering, FOS: Electrical engineering, electronic engineering, information engineering},
    title = {Piezoelectric Soft Robot Inchworm Motion by Controlling Ground Friction through Robot Shape},
    publisher = {arXiv},
    year = {2021},
    copyright = {arXiv.org perpetual, non-exclusive license}
}
```
