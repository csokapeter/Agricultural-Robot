# Agricultural Robot Project

This repository contains code for the Agricultural Robot simulation using ML-Agents and PyTorch. Follow the steps below to set up the environment and run the project.

## Prerequisites

To set up the project, you’ll need to create a Conda environment with Python 3.9, and install all the necessary dependencies.

### 1. Create and Activate the Conda Environment

First, create a new Conda environment named `agricultural-robot` with Python 3.9:
```bash
conda create -n agricultural-robot python=3.9
```
Activate the environment:
```bash
conda activate agricultural-robot
```
### 2. Install ML-Agents (Version 0.29.0)
Install the ML-Agents package (version 0.29.0) using pip:
```bash
pip install mlagents==0.29.0
```
### 3. Install Compatible PyTorch Version
The project uses PyTorch with CUDA 11.3 support. Install a compatible version of PyTorch, torchvision, and torchaudio using the following command:
```bash
pip install torch==1.12.1+cu113 torchvision==0.13.1+cu113 torchaudio==0.12.1 --extra-index-url https://download.pytorch.org/whl/cu113
```
### 4. Downgrade protobuf to Version 3.19.4
There’s a known compatibility issue with newer versions of protobuf. Downgrade protobuf to version 3.19.4:
```bash
pip install protobuf==3.19.4
```

## Unity
The simulation was created using Unity version 2022.3.20f1, but it should work with newer versions.
You can download Unity from the [official website](https://unity.com/download).

## Running the project
1. Open the simulation in Unity and load the **SampleScene**.  
2. Run `scout_agent.py` using the appropriate command for your operating system:  
   - **Windows:**  
   ```bash
   python scout_agent.py
   ```  
   - **Linux/macOS:**  
   ```bash
   python3 scout_agent.py
   ``` 
   - To see available options, run:  
   ```bash
   python scout_agent.py --help
   ```
   Wait until you see the message: **"Loading Unity Environment..."**.
3. Start the simulation by clicking the **Play** button in Unity.  
