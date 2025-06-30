#!/bin/bash

# Robot State Monitor Startup Script

echo "Robot State Monitor Tools"
echo "========================="
echo "1. Real-time Monitor & Plotting"
echo "2. Data Recording"
echo "3. Data Analysis"
echo "4. Exit"
echo ""

read -p "Please select function (1-4): " choice

case $choice in
    1)
        echo "Starting real-time monitor and plotting..."
        python3 $(dirname "$0")/robot_state_monitor.py
        ;;
    2)
        echo "Starting data recording..."
        echo "Press Ctrl+C to stop recording"
        python3 $(dirname "$0")/data_logger.py
        ;;
    3)
        echo "Starting data analysis..."
        python3 $(dirname "$0")/analyze_data.py
        ;;
    4)
        echo "Exit"
        exit 0
        ;;
    *)
        echo "Invalid selection"
        exit 1
        ;;
esac 