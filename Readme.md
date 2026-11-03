# Aurora Control Framework

ROS-based framework for processing Aurora sensor data and running closed-loop control (PID/MPC) on a soft robotic system.

**Contact:**  
Filippo Manini — filippo.manini@mail.polimi.it

## Overview
This repository contains the processing, estimation, and control pipeline used to:
- read Aurora tracking data
- estimate Jacobians and trajectories
- run closed-loop control (PID / MPC)
- interface with an Arduino-based actuation system

The system is configured via a YAML file and instantiated through a custom configuration parser.

## Architecture (high-level)
Main components:
- `processor`: sensor processing and state estimation
- `communicator`: Arduino / hardware interface
- `controller`: closed-loop control logic (PID / MPC)
- `trajectory_estimator`, `jacobian_estimator`, `valve_predictor`: estimation modules

Components are instantiated from configuration and wired together at runtime.

## Requirements
- Python 3
- ROS (tested with Noetic)
- numpy, scipy, rospy

## How to run

Make sure ROS is sourced before running:

```bash
cd <path_to_controller_directory>
./scripts/run_controller.sh

















Note sulla calibrazione:

La calibrazione corretta se:
1. la X è positiva su posteriore ( S )
2. la Y è negativa sul Medio ( M )
3. La Z è positiva su Inserzione ( I ) 

Se è così la tertna è destrorsa e corretta. Altrimenti occorre cambiare il segno quando si calcola: 
normal =  self.process_aurora_data(aurora_tip_data, aurora_base_data) 
in controller.processor.calibrate_aurora_data ( il file si trova in src/nodes nella classe usata come processor nel config.yaml)

 
