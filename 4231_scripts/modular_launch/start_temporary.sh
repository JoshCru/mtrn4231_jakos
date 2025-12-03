#!/bin/bash
# =============================================================================
# Temporary Nodes Launcher
#
# Launches temporary/one-off nodes that can be started and stopped:
#   - Go Home
#   - Perception (Real/Simulated)
#   - Weight Detection (Real/Simulated/Off)
#   - Position Check (for simulated perception)
#   - Application Mode (Sorting System or Simple Pick & Weigh)
#
# Usage:
#   ./start_temporary.sh [OPTIONS]
#
# Options:
#   --mode MODE           Application mode: "sorting" or "simple" (default: sorting)
#   --sim-perception      Use simulated perception
#   --real-perception     Use real perception (camera-based)
#   --sim-weight          Use simulated weight detection (default)
#   --real-weight         Use real weight detection
#   --no-weight           Skip weight detection entirely
#   --weight-detector-cpp Use C++ weight detector implementation (default)
#   --weight-detector-python  Use Python weight detector implementation
#   --go-home             Run go_home action before other nodes
#   --position-check      Run position check (for simulated perception)
#   --autorun             Auto-start application after launch
#   --grip-weight GRAMS   Weight for gripper angle (simple mode only, default: 100)
#   --help                Show this help message
#
# Examples:
#   ./start_temporary.sh --mode sorting --sim-perception --position-check --autorun
#   ./start_temporary.sh --mode simple --real-weight --go-home --grip-weight 150
#   ./start_temporary.sh --mode sorting --real-perception --real-weight
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../../ros2_system"

# Default configuration
APP_MODE="sorting"  # Options: "sorting", "simple"
SIM_PERCEPTION=false
WEIGHT_MODE="sim"  # Options: "sim", "real", "none"
WEIGHT_DETECTOR_IMPL="cpp"  # Options: "cpp", "python"
RUN_GO_HOME=false
RUN_POSITION_CHECK=false
AUTORUN=false
GRIP_WEIGHT=100

# Parse arguments
show_help() {
    head -n 33 "$0" | tail -n 29
    exit 0
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --mode|-m)
            APP_MODE="$2"
            shift 2
            ;;
        --sim-perception)
            SIM_PERCEPTION=true
            shift
            ;;
        --real-perception)
            SIM_PERCEPTION=false
            shift
            ;;
        --sim-weight)
            WEIGHT_MODE="sim"
            shift
            ;;
        --real-weight)
            WEIGHT_MODE="real"
            shift
            ;;
        --no-weight)
            WEIGHT_MODE="none"
            shift
            ;;
        --weight-detector-cpp)
            WEIGHT_DETECTOR_IMPL="cpp"
            shift
            ;;
        --weight-detector-python)
            WEIGHT_DETECTOR_IMPL="python"
            shift
            ;;
        --go-home)
            RUN_GO_HOME=true
            shift
            ;;
        --position-check)
            RUN_POSITION_CHECK=true
            shift
            ;;
        --autorun)
            AUTORUN=true
            shift
            ;;
        --grip-weight|-g)
            GRIP_WEIGHT="$2"
            shift 2
            ;;
        --help|-h)
            show_help
            ;;
        *)
            shift
            ;;
    esac
done

# Validate app mode
if [[ "$APP_MODE" != "sorting" && "$APP_MODE" != "simple" ]]; then
    echo "ERROR: Invalid mode '$APP_MODE'. Must be 'sorting' or 'simple'."
    exit 1
fi

echo "=========================================="
echo "   TEMPORARY NODES LAUNCHER"
echo "=========================================="
echo "  Application Mode: $(echo $APP_MODE | tr '[:lower:]' '[:upper:]')"
echo "  Perception Mode:  $([ "$SIM_PERCEPTION" = true ] && echo "SIMULATED" || echo "REAL")"
echo "  Weight Mode:      $(echo $WEIGHT_MODE | tr '[:lower:]' '[:upper:]')"
echo "  Go Home:          $RUN_GO_HOME"
echo "  Position Check:   $RUN_POSITION_CHECK"
echo "  Autorun:          $AUTORUN"
[ "$APP_MODE" = "simple" ] && echo "  Grip Weight:      ${GRIP_WEIGHT}g"
echo "=========================================="
echo ""

source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

# Track PIDs for cleanup
PIDS=()
STEP=1

# 1. Go Home (optional)
if [ "$RUN_GO_HOME" = true ]; then
    echo "[$STEP] Moving robot to HOME position..."
    ros2 run motion_control_package go_home 5.0
    echo "  Robot at home position"
    sleep 2
    echo ""
    STEP=$((STEP + 1))
fi

# 2. Perception (optional - only if enabled)
PERCEPTION_PID=""
if [ "$SIM_PERCEPTION" = true ]; then
    echo "[$STEP] Starting Simulated Perception..."
    ros2 run perception_package simulated_perception_node \
        --ros-args \
        -p num_objects:=4 \
        -p publish_rate:=5.0 \
        -p randomize_positions:=true &
    PERCEPTION_PID=$!
    PIDS+=($PERCEPTION_PID)
    sleep 2
    echo ""
    STEP=$((STEP + 1))
fi

# 3. Weight Detection
WEIGHT_PID=""
PLOT_PID=""

case $WEIGHT_MODE in
    real)
        echo "[$STEP] Starting Real Weight Detection ($WEIGHT_DETECTOR_IMPL)..."
        if [ "$WEIGHT_DETECTOR_IMPL" = "python" ]; then
            ros2 run weight_detection_package weight_detector_py &
        else
            ros2 run weight_detection_package weight_detector &
        fi
        WEIGHT_PID=$!
        PIDS+=($WEIGHT_PID)
        sleep 2

        echo "  Starting PlotJuggler for weight visualization..."
        "${ROS2_WS}/plot_weight.sh" &
        PLOT_PID=$!
        PIDS+=($PLOT_PID)
        sleep 2
        echo ""
        STEP=$((STEP + 1))
        ;;
    sim)
        echo "[$STEP] Weight Detection: SIMULATED (using simulated weights)"
        echo ""
        STEP=$((STEP + 1))
        ;;
    none)
        echo "[$STEP] Weight Detection: DISABLED"
        echo ""
        STEP=$((STEP + 1))
        ;;
esac

# 4. Position Check (optional, for simulated perception)
if [ "$RUN_POSITION_CHECK" = true ]; then
    if [ "$SIM_PERCEPTION" = true ]; then
        echo "[$STEP] Running Position Check for Simulated Perception..."
        echo "  The robot will visit each simulated weight position at Z_PICKUP."
        echo ""
        python3 "${ROS2_WS}/src/motion_control_package/scripts/check_simulated_positions.py"

        if [ $? -eq 0 ]; then
            echo ""
            echo "  Position check completed successfully!"
            echo ""
        else
            echo ""
            echo "  Position check failed or was cancelled!"
            read -p "  Continue anyway? (y/n): " continue_response
            if [[ ! $continue_response =~ ^[Yy]$ ]]; then
                echo "  Stopping..."
                for pid in "${PIDS[@]}"; do
                    kill $pid 2>/dev/null || true
                done
                exit 1
            fi
        fi
        STEP=$((STEP + 1))
        echo ""
    else
        echo "[$STEP] Skipping position check (only applicable for simulated perception)"
        STEP=$((STEP + 1))
        echo ""
    fi
fi

# 5. Application Mode - Sorting or Simple Pick & Weigh
APP_PID=""

if [ "$APP_MODE" = "sorting" ]; then
    echo "[$STEP] Starting Sorting Brain..."
    ros2 run supervisor_package sorting_brain_node &
    APP_PID=$!
    PIDS+=($APP_PID)
    sleep 2
    echo ""
    STEP=$((STEP + 1))

    if [ "$AUTORUN" = true ]; then
        echo "AUTORUN: Starting sorting in 2 seconds..."
        sleep 2
        ros2 topic pub --once /sorting/command std_msgs/msg/String "data: 'start'"
        echo "  Sorting started!"
        echo ""
    fi

elif [ "$APP_MODE" = "simple" ]; then
    echo "[$STEP] Running Simple Pick and Weigh..."
    echo "  Grip Weight: ${GRIP_WEIGHT}g"
    echo ""

    # Run simple pick and weigh (blocking call)
    ros2 run motion_control_package simple_pick_and_weigh_node \
        --ros-args \
        -p grip_weight:=${GRIP_WEIGHT} \
        -p initial_positioning:=true

    echo ""
    echo "Pick and weigh complete!"
    echo ""
fi

echo "=========================================="
echo "   Temporary nodes status"
echo "=========================================="
echo ""
echo "Running nodes:"
[ -n "$PERCEPTION_PID" ] && echo "  - Simulated Perception (PID: $PERCEPTION_PID)"
[ -n "$WEIGHT_PID" ] && echo "  - Weight Detection (PID: $WEIGHT_PID)"
[ -n "$PLOT_PID" ] && echo "  - PlotJuggler (PID: $PLOT_PID)"
[ -n "$APP_PID" ] && echo "  - Sorting Brain (PID: $APP_PID)"
[ "$APP_MODE" = "simple" ] && echo "  - Simple Pick & Weigh (completed)"
echo ""

if [ "${#PIDS[@]}" -gt 0 ]; then
    echo "Press Ctrl+C to stop all temporary nodes..."
    echo ""

    # Cleanup function
    cleanup() {
        echo ""
        echo "Shutting down all temporary nodes..."
        for pid in "${PIDS[@]}"; do
            kill $pid 2>/dev/null || true
        done
        echo "Done."
        exit 0
    }

    trap cleanup SIGINT SIGTERM

    # Wait for any process to exit
    wait -n

    # If we get here, one process died unexpectedly
    echo ""
    echo "WARNING: A node has stopped unexpectedly!"
    cleanup
else
    echo "No background nodes launched. Tasks completed."
fi
