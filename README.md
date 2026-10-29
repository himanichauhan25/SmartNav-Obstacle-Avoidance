# SmartNav AI Obstacle Avoidance

## Project Objective
This project is a 2D virtual simulation of an autonomous mobile robot (SmartNav) designed for an Artificial Intelligence and Data Science undergraduate project. It demonstrates path planning, obstacle avoidance, human detection, sensor fusion, and decision-making logic using a deterministic scenario.

## Features
- **Simulated Differential Drive Robot**: Navigates autonomously towards a goal.
- **Simulated Ultrasonic Sensors**: Uses raycasting to calculate distances to walls, humans, and static obstacles.
- **Simulated RGB Camera & AI Perception**: Detects objects in a 70-degree field-of-view, labeling them as HUMAN or STATIC OBSTACLE with simulated confidence scores.
- **Sensor Fusion**: Combines ultrasonic distances with AI perception to build a unified environmental state.
- **Decision-Making Layer**: Evaluates risks and decides whether to move forward, turn, slow down, stop, or reverse.
- **Safety Layer**: Overrides normal navigation to trigger an EMERGENCY STOP if a collision is imminent.
- **Live Dashboard**: Displays real-time metrics, system states, and sensor data.

## System Architecture
The system consists of the following modules integrated into a single simulation loop:
- **Environment**: Manages walls, static obstacles, moving humans, and the goal.
- **Sensors**: Raycasting engine for ultrasonics and conical intersection for the simulated camera.
- **AI Perception (SIMULATED)**: Classifies entities within the camera FOV.
- **Sensor Fusion**: Aggregates sensor data for the decision engine.
- **Navigation Engine**: Determines the safest heading to avoid obstacles while progressing to the goal.

## Academic Note
**Important:** AI perception, camera sensing, ultrasonic sensing, and Raspberry Pi processing are represented as simulated components within the virtual prototype. This project does not use real-world YOLO inference or hardware measurements, but instead demonstrates the proposed architecture and decision-making logic of the SmartNav system.

## How to Install
1. Ensure you have Python 3 installed.
2. Open a terminal in this directory.
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## How to Run
Run the main script:
```bash
python main.py
```
The simulation will start automatically and run the deterministic demonstration scenario.

## Controls
- **SPACE**: Pause / Resume simulation
- **R**: Reset simulation
- **ESC**: Exit
- **D**: Toggle dashboard visibility
- **S**: Toggle ultrasonic sensor rays visibility
- **C**: Toggle camera field-of-view (FOV) visibility
- **A**: Toggle AI perception visualization (bounding boxes and labels)
