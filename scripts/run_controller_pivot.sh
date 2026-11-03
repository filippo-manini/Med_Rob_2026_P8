#!/bin/bash

AURORA_PATH=~/catkin_ws/src/filippo_controller/ndi_aurora_2_6_x
CONTROLLER_PATH=~/catkin_ws
RVIZ_CONFIG=$CONTROLLER_PATH/src/filippo_controller/rviz/controller_visualizator.rviz
ARDUINO_PATH=$CONTROLLER_PATH/src/filippo_controller/arduino_code/arduino_communication


# load the code on the arduino
# Trova automaticamente Arduino tra le porte seriali
DEVTTY=$(ls /dev/ttyACM* 2>/dev/null | head -n 1)

if [[ -z "$DEVTTY" ]]; then
    echo "Errore: nessun Arduino trovato su ttyACM*"
    exit 1
else
    echo "Arduino trovato su $DEVTTY"
fi


echo "loading arduino code from folder $ARDUINO_PATH"
# Compile Arduino sketch
arduino-cli compile --fqbn arduino:avr:uno $ARDUINO_PATH/arduino_communication.ino || { echo "Compile failed"; exit 1; }

# Upload Arduino sketch
arduino-cli upload -p $DEVTTY --fqbn arduino:avr:uno $ARDUINO_PATH/arduino_communication.ino || { echo "Upload failed"; exit 1; }

echo "Upload completed, starting ROS..."





# Start roscore
gnome-terminal -- bash -c "echo 'Starting roscore...'; roscore; exec bash"

# Wait for roscore
until rostopic list > /dev/null 2>&1; do
    echo "Waiting for roscore to start..."
    sleep 1
done

# Launch Aurora
if [[ ! -x "$AURORA_PATH/run.sh" ]]; then
    echo "Error: $AURORA_PATH/run.sh not found or not executable!"
    exit 1
fi
gnome-terminal -- bash -c "cd $AURORA_PATH && ./run_pivot.sh; exec bash"


gnome-terminal -- bash -c "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200; exec bash"
# Wait for /aurora_data
echo "Waiting for /aurora_data topic to publish data..."
until rostopic echo -n1 /aurora_data 2>/dev/null | grep . >/dev/null; do
    sleep 0.5
done
echo "/aurora_data is publishing now!"

# Echo aurora_data
gnome-terminal -- bash -c "rostopic echo /aurora_data; exec bash"

# Launch controller
gnome-terminal -- bash -c "cd $CONTROLLER_PATH && source devel/setup.bash && rosrun filippo_controller main.py; exec bash"

# Launch RViz with config
if [[ -f "$RVIZ_CONFIG" ]]; then
    gnome-terminal -- bash -c "rviz -d $RVIZ_CONFIG;"
else
    echo "Warning: RViz config not found at $RVIZ_CONFIG, launching default RViz."
    gnome-terminal -- bash -c "rviz;"
fi

