# CSC376 Project: Checkers Robot

University of Toronto Mississauga \
CSC376H5 F *Fundamentals of Robotics* \
2025 Fall Semester

This repository contains the code used for the final course project of CSC376 at the University of Toronto Mississauga. The project commands a Franka Emika Panda robotic arm and an Intel RealSense USB depth camera to play a game of checkers against a human opponent.

Modes:
1. AI vs AI
2. Human vs AI

## Environment Setup

1. Buy a Franka Emika Panada.
2. Install Franka Desk and [miniconda](https://conda.io/projects/conda/en/latest/index.html).
3. Create virtual environment and install dependencies
   ```bash
   conda create -n checkers-robot python=3.12
   conda activate checkers-robot
   pip3 install -r requirements.txt
   ```

## Run
```bash
QT_QPA_PLATFORM=xcb python3 main.py <camera_index>
```

Replace `<camera_index>` with the index of your RealSense camera (usually 4).

Check `ls /dev/video*` to list all available indexes.