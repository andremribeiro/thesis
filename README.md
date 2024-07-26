# Thesis Repository
Welcome to the repository for my thesis project. This repository contains all the code, data, and documentation related to my research.

## Table of Contents
- Introduction
- Installation
- Usage
- Contact

## Introduction
This project is focused on Cooperative 3D Exploration using Distributed Multi-UAV Teams. The repository includes all necessary scripts, datasets, and documentation to reproduce the results presented in the thesis.

## Installation

### Prerequisites
- Ubuntu 20.04
- ROS Noetic
- [MRS UAV Simulator](https://github.com/ctu-mrs/mrs_uav_system)

### Clone the Repository and Mean-Shift Clustering
```bash
mkdir -p ~/exploration_ws/src
cd ~/exploration_ws/src
git clone https://github.com/andremribeiro/thesis.git
git clone https://github.com/andremribeiro/mean_shift_clustering.git
cd ..
catkin build
source devel/setup.bash
```

### OctoMap and Path Planner
```bash
mkdir -p ~/mapping_ws/src
cd ~/mapping_ws/src
git clone https://github.com/andremribeiro/octomap_mapping.git
git clone https://github.com/andremribeiro/mrs_octomap_planner.git
cd ..
catkin build
source devel/setup.bash
```

## Usage
### Running the simulation
For changing between teams of one, two, and three UAVs change the session.yml in start.sh to the desired number of UAVs
```bash
cd ~/exploration_ws/src/thesis/tmux
./start.sh
```

## Contact
For any questions or inquiries, please contact me at [andre.m.ribeiro.2000@tecnico.ulisboa.pt].

Feel free to edit and expand on each section to better suit the specifics of your thesis project.
