# Implementation of swarm robots for ENPM808X final project
[![License: MIT](https://img.shields.io/badge/License-MIT-red.svg)](https://opensource.org/licenses/MIT)
![CICD Workflow status](https://github.com/Sameer-Arjun-S/hydra/actions/workflows/cmake.yml/badge.svg)
[![codecov](https://codecov.io/gh/Sameer-Arjun-S/hydra/branch/development_branch/graph/badge.svg)](https://codecov.io/gh/Sameer-Arjun-S/hydra)


# Authors:
- [Sameer Arjun S](https://github.com/Sameer-Arjun-S) 
- [Manav Bhavesh Nagda](https://github.com/mvboiii)
- [Ishaan Samir Parikh](https://github.com/Ishaan1810)


# Introduction
Acme Robotics aims to revolutionize fire response systems within warehouse facilities susceptible to
fire incidents by introducing ”Project Hydra” a swarm of turtle bots designed to contain
and address fire emergencies effectively. This innovative solution utilizes a fleet of 20 turtlebots equipped with
waterjet spraying capabilities to encircle and contain fire outbreaks within warehouse environments.
The Hydra project employs robots designed to respond autonomously to fire emergencies
within the facility. The project targets a unique feature to surround the affected machine/object as
the most optimal formation to prevent any fire hazards. In the event of a fire outbreak, the system
receives input specifying the affected machine. Later, the robots approach the destination and form
the optimal shape of surroundings in square, circle, or triangle shapes.

# Agile Implpementation Process:
- [Agile Iterative Process](https://docs.google.com/spreadsheets/d/1kNnjrfgtdtyvd8Hb7R8aV4-XI82XxSHm4BPfusI4Z_g/edit#gid=1106376998)
- [Sprint Review](https://docs.google.com/document/d/1fweS0_-lTLAzGLIpkdcCDKc-3AHm12xAb0H6oILBxqQ/edit)

# Report
- [Phase 0 Report](https://github.com/Sameer-Arjun-S/hydra/blob/development_branch/Phase%200%20Report.pdf)

# Phase 1 presentation video
[Video](https://drive.google.com/drive/folders/1NMGBkv37AdHuSnkcF0G6__kuWJ8FNRUv?usp=drive_link)


## Compiling and running via command line:
```
#Cloning the repository
  git clone https://github.com/Sameer-Arjun-S/hydra 
#Configure the project and generate a native build system
  cmake -S ./ -B build/
#Compiling and building the project
  cmake --build build/ --clean-first
# Build the documentation into the 'docs' directory using CMake:
  cmake --build build/ --hydra

```
