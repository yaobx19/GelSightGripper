# Tactile Gripper Control System

A tactile sensing and gripper control system that integrates five core perception capabilities for intelligent object manipulation.

## Features

### 1. Contact Detection (接触识别)
- Monitors tactile sensor displacement to determine object contact
- Triggers gripper activation when stable contact is detected

### 2. Slip Detection (滑移检测)
- **X-direction slip**: Detects lateral slipping, triggers grip release
- **Y-direction slip**: Detects vertical slipping, provides warnings

### 3. Weight Estimation (重量估计)
- Calculates liquid volume based on tactile sensor displacement
- Real-time liquid level monitoring

### 4. Disturbance Identification (扰动识别)
- Distinguishes between intentional manipulation and external disturbances
- Prevents false alarms during normal handling

### 5. Motion Recognition (动作识别)
- Detects rolling/twisting motions (e.g., bottle cap opening)
- Enables context-aware gripper responses

## System Components

- **`main.py`**: Main control loop with state machine logic
- **`gripper.py`**: Electric gripper controller (ModBus RTU)
- **`GSmini.py`**: Tactile sensor interface and processing

## Hardware Requirements

- Electric gripper with ModBus RTU interface
- Dual GelSight Mini tactile sensors
- Serial port (default: COM3)

## Usage

```bash
python main.py
