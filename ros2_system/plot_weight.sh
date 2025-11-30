#!/bin/bash
# Launch PlotJuggler with weight estimator configuration

# Change to the script's directory
cd "$(dirname "$0")"

# Wait for topic to be available
echo "Waiting for /estimated_mass topic..."
timeout 10 bash -c 'until ros2 topic list | grep -q "/estimated_mass"; do sleep 0.5; done'

if [ $? -eq 0 ]; then
    echo "Topic found! Launching PlotJuggler..."
    ros2 run plotjuggler plotjuggler --layout weight_estimator_cfg.xml
else
    echo "Warning: /estimated_mass topic not found. Launching anyway..."
    ros2 run plotjuggler plotjuggler --layout weight_estimator_cfg.xml
fi
