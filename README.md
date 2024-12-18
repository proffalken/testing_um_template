# testing_um_template
This is a test of the Urban Machine template

---
[![Test Status](https://github.com/proffalken/testing_um_template/workflows/Test/badge.svg)](https://github.com/proffalken/testing_um_template/actions?query=workflow%3ATest)
[![Lint Status](https://github.com/proffalken/testing_um_template/workflows/Lint/badge.svg)](https://github.com/proffalken/testing_um_template/actions?query=workflow%3ALint)
[![codecov](https://codecov.io/gh/proffalken/testing_um_template/branch/main/graph/badge.svg)](https://codecov.io/gh/proffalken/testing_um_template)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)
![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?logo=docker&logoColor=white)
![ROS2](https://img.shields.io/badge/ros-%230A0FF9.svg?logo=ros&logoColor=white)

---

## Running This Project

To run the project, use the following command:

```shell
docker/launch umtemplate_launch_profile
```
Then, open http://localhost/ on your browser to view the project logs.

For in-depth documentation on the repository features, read the [About Template](docs/about_template.md) documentation.

### Dependencies

- [Docker](https://docs.docker.com/get-docker/), and optionally [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) for hardware acceleration.
- [Poetry](https://python-poetry.org/docs/), in order to use linting tooling.

---
This repository was initialized by the [create-ros-app](https://github.com/UrbanMachine/create-ros-app) template. Contributions are welcome!
