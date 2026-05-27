# TurtleBot3 Autonomous Navigation & Victim Detection

This repository contains the source code for an autonomous navigation and detection system developed for a TurtleBot3 (Burger). The project was created as a 2nd-semester project in Computer Technology at Aarhus University (AU).

The system enables the robot to autonomously navigate a simulated disaster zone, avoid obstacles, and identify victims (red markers on the floor) using color recognition.

## Features

* **FSM Navigation:** The robot is controlled by a Finite State Machine (FSM) utilizing four states to ensure continuous movement and prevent it from getting stuck in corners or dead ends.
* **Proportional Control Logic:** Employs a variable turning ratio to center the robot on its path. The system maintains an average linear speed of approximately 0.16 m/s.
* **Asynchronous Victim Detection:** The RGB sensor is polled asynchronously at 10 Hz in a dedicated thread. This ensures that color recognition processing does not delay the primary motor control and obstacle avoidance loops.
* **Auto-Recovery:** A built-in failsafe mechanism takes over if the LiDAR sensor is blinded (e.g., if it returns over 50% NaN values due to proximity constraints).

## Hardware & Sensors

* **Platform:** TurtleBot3 Burger equipped with a Raspberry Pi 4 (Ubuntu 22.04).
* **LiDAR:** LDS-02 (360-degree scanning, divided into dynamic slices for front, sides, and shoulders).
* **Color Sensor:** ISL29125 RGB sensor (I2C communication directly via Raspberry Pi GPIO).
* **Visual Feedback:** Top-mounted LED (GPIO 17) for visual indication of identified victims.

## System Architecture (FSM)

The core navigation logic is structured as a Finite State Machine that evaluates real-time LiDAR data to determine the robot's behavior.

```mermaid
stateDiagram-v2
    direction TB
    
    %% States
    state "Clear Line" as Clear
    state "Warning Zone" as Warning
    state "V-Shape Mode" as VShape
    state "Recovery Mode" as Recovery

    [*] --> Clear : Start Node

    %% Normal transitions
    Clear --> Warning : Front < 0.2m AND (Left OR Right < 0.2m)
    Clear --> VShape : Front, Left AND Right < 0.2m
    
    Warning --> Clear : Path ahead is clear
    Warning --> VShape : Front, Left AND Right < 0.2m
    
    VShape --> Clear : Rotated clear (Path open)
    VShape --> Warning : Rotated partially clear

    %% Recovery Mode overrides
    Clear --> Recovery : NaN readings > 50%
    Warning --> Recovery : NaN readings > 50%
    VShape --> Recovery : NaN readings > 50%

    Recovery --> Clear : NaN readings <= 30%