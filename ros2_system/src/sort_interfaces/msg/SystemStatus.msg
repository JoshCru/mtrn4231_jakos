# System Status Message - Published by sorting_brain_node and simple_pick_and_weigh_node
# Used for monitoring, debugging, and visualization in dashboard

# Timestamp
builtin_interfaces/Time timestamp

# Node identifier
string node_name              # e.g., "sorting_brain", "simple_pick_and_weigh"

# Current state/phase
string state                  # e.g., "IDLE", "MOVING", "GRIPPING", "WEIGHING", "PLACING"

# Detailed status message
string status_message         # Human-readable description of current action

# Progress information
int32 current_step            # Current step number in sequence
int32 total_steps             # Total number of steps (0 if unknown)
float32 progress_percent      # Progress percentage (0-100)

# Current object information (if applicable)
int32 object_id               # Object being processed (-1 if none)
int32 object_weight           # Object weight in grams (-1 if unknown)
string target_bin             # Target bin name (empty if none)

# Flags for specific activities
bool is_moving                # Robot is in motion
bool is_gripping              # Gripper is actuating
bool is_weighing              # Weight measurement in progress
bool is_waiting               # Waiting for external condition

# Error/warning information
bool has_error                # Error condition present
bool has_warning              # Warning condition present
string error_message          # Error description (empty if no error)
