# Recognition Module Communication Pattern

## Overview

The recognition module uses a **Publisher-Subscriber** pattern to communicate with the sort_node and other system components.

## Communication Flow

```
Point Cloud → Recognition Node → Sort Node → Planning
```

### Detailed Flow

1. **Input**: Recognition Node subscribes to point cloud data
   - Topic: `/camera/pointcloud`
   - Message Type: `sensor_msgs/PointCloud2`
   - Source: Camera node or simulation

2. **Processing**: Recognition Node analyzes point cloud
   - Filters workspace
   - Clusters objects
   - Calculates volumes and weights
   - Estimates object properties

3. **Output**: Recognition Node publishes weight estimates
   - Topic: `/recognition/estimated_weights`
   - Message Type: `sort_interfaces/WeightEstimate`
   - Subscriber: Sort Node

4. **Decision**: Sort Node makes sorting decisions
   - Topic: `/planning/sort_decisions`
   - Message Type: `sort_interfaces/SortDecision`
   - Subscriber: Planning/execution nodes

## Message Formats

### WeightEstimate (Recognition → Sort)

```
std_msgs/Header header
uint32 object_id              # Unique ID for each detected object
float32 estimated_weight      # Weight in grams
float32 confidence            # 0.0 to 1.0
geometry_msgs/Pose pose       # Position (x, y, z) and orientation
float32 volume                # Volume in cm³
```

**Fields Explained:**
- `object_id`: Incremental counter, resets on node restart
- `estimated_weight`: Calculated from volume × material density (8000 kg/m³ for stainless steel)
- `confidence`: Based on cluster size (more points = higher confidence)
- `pose`: Object centroid position in camera frame
- `volume`: Convex hull volume in cubic centimeters

## Recognition Node Output

### Simplified Console Output

The recognition node now provides concise, easy-to-read output:

```
Detected 3 object(s)
  Object 0: 50.0g, 2.5×2.5×1.0cm @ (0.30, 0.15, 0.10)
  Object 1: 100.0g, 3.0×3.0×1.5cm @ (0.35, -0.10, 0.12)
  Object 2: 200.0g, 4.0×4.0×1.5cm @ (0.25, 0.00, 0.12)
```

**Format**: `Object ID: weight, width×height×depth @ (x, y, z)`

This makes it easy to:
- Track multiple objects at a glance
- Compare detected vs expected weights
- Monitor object positions
- Debug recognition issues

## Communication Pattern: Publisher-Subscriber

### Why Publisher-Subscriber?

1. **Decoupling**: Recognition and sorting are independent
2. **Scalability**: Multiple nodes can subscribe to weight estimates
3. **Reliability**: Messages are buffered (queue size: 10)
4. **Real-time**: Low latency communication
5. **ROS2 Standard**: Native ROS2 pattern

### Advantages

✓ **Loose Coupling**: Nodes can be developed/tested independently
✓ **Flexibility**: Easy to add new subscribers (e.g., visualization, logging)
✓ **Reliability**: QoS policies ensure message delivery
✓ **Performance**: Direct memory sharing for same-process nodes

### Data Flow Example

```
Time  | Recognition Node                    | Sort Node
------|-------------------------------------|-----------------------------
t=0   | Receives point cloud                |
t=1   | Detects 3 objects                   |
t=2   | Publishes WeightEstimate (Object 0) | Receives estimate
t=2   | Publishes WeightEstimate (Object 1) | Makes decision for Object 0
t=2   | Publishes WeightEstimate (Object 2) | Publishes SortDecision
t=3   |                                     | Makes decision for Object 1
t=3   |                                     | Publishes SortDecision
t=4   | Receives next point cloud           | Makes decision for Object 2
...
```

## Testing Communication

### Check Topics

```bash
# List all active topics
ros2 topic list

# Should show:
# /camera/pointcloud
# /recognition/estimated_weights
# /planning/sort_decisions
```

### Monitor Messages

```bash
# View point cloud rate
ros2 topic hz /camera/pointcloud

# View weight estimates
ros2 topic echo /recognition/estimated_weights

# View sort decisions (if sort_node running)
ros2 topic echo /planning/sort_decisions
```

### Check Connections

```bash
# See who publishes to recognition
ros2 topic info /camera/pointcloud

# See who subscribes to recognition output
ros2 topic info /recognition/estimated_weights
```

## Integration with Sort Node

The sort_node subscribes to `/recognition/estimated_weights` and:

1. **Receives** each WeightEstimate message
2. **Matches** weight to target areas (e.g., 50g bin, 100g bin, etc.)
3. **Creates** SortDecision with:
   - Object ID
   - Target area ID
   - Pick pose (from recognition)
   - Place pose (target area location)
   - Sorting strategy reason

4. **Publishes** to `/planning/sort_decisions`

### Sort Node Callback

```cpp
void weight_estimate_callback(WeightEstimate::SharedPtr msg) {
    // Make sorting decision based on weight
    auto decision = make_sorting_decision(*msg);

    // Publish decision
    sort_decisions_pub_->publish(*decision);

    // Log decision
    RCLCPP_INFO("Object %u (%.1fg) → Target Area %u",
                msg->object_id, msg->estimated_weight,
                decision->target_area_id);
}
```

## Quality of Service (QoS)

Both topics use **default QoS** with:
- Queue size: 10
- Reliability: Reliable (guaranteed delivery)
- Durability: Volatile (no message history)
- History: Keep last 10 messages

This ensures:
- No message loss under normal conditions
- Bounded memory usage
- Real-time performance

## Error Handling

### Recognition Node
- Empty point cloud → Skip processing
- No objects detected → Log debug message
- Invalid volume → Warn and skip object
- Low confidence → Skip object (below threshold)

### Sort Node
- No target areas defined → Warn and wait
- No matching target → Use fallback strategy
- Multiple matches → Choose best match

## Performance Considerations

### Message Frequency
- Point clouds: Typically 1-30 Hz
- Weight estimates: Per-object (3 messages for 3 objects)
- Sort decisions: Per-object

### Latency
- Recognition processing: ~50-200ms per point cloud
- Message transmission: <1ms (same machine)
- Total pipeline latency: ~100-250ms

### Throughput
- Can handle 10+ objects per second
- Scales with point cloud complexity
- Bottleneck: Recognition processing (PCL algorithms)

## Summary

**Pattern**: Publisher-Subscriber (ROS2 standard)

**Topic**: `/recognition/estimated_weights`

**Message**: `sort_interfaces/WeightEstimate`
- Contains: ID, weight, confidence, pose, volume

**Output**: Simple, concise format for easy debugging
```
Detected N object(s)
  Object ID: weight, size @ position
```

**Next Step**: Sort node processes estimates and publishes decisions

This design enables:
✓ Modular architecture
✓ Easy testing and debugging
✓ Scalable to multiple subscribers
✓ Real-time performance
✓ Clear data flow
