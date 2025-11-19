#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Function to generate a collision object (box)
auto generateCollisionObject(float sx, float sy, float sz, float x, float y, float z,
                              std::string frame_id, std::string id) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = sx;
  primitive.dimensions[primitive.BOX_Y] = sy;
  primitive.dimensions[primitive.BOX_Z] = sz;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = x;
  box_pose.position.y = y;
  box_pose.position.z = z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}

class SafetyBoundaryNode : public rclcpp::Node
{
public:
  SafetyBoundaryNode() : Node("safety_boundary_node")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Safety Boundary Node...");

    // Create MoveGroupInterface
    move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "ur_manipulator");

    std::string frame_id = move_group_interface->getPlanningFrame();
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", frame_id.c_str());

    // Create PlanningSceneInterface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Safety boundaries based on Hessian normal form:
    // Table: [0, 0, 1, 0] -> z = 0
    // Back_Plane: [0, -1, 0, 0.3] -> y = -0.3
    // Side_Plane: [-1, 0, 0, 0.3] -> x = -0.3
    // Ceiling: z = 0.655

    // Table boundary (z < 0) - box below the table
    auto col_object_table = generateCollisionObject(
      4.0, 4.0, 0.5,           // size: 4m x 4m x 0.5m thick
      0.0, 0.0, -0.25,         // center at z=-0.25m
      frame_id, "table_boundary"
    );
    RCLCPP_INFO(this->get_logger(), "Adding table boundary at z=0");

    // Back wall (y < -0.3) - wall behind the robot
    auto col_object_back = generateCollisionObject(
      4.0, 0.1, 1.0,           // size: 4m wide x 0.1m thick x 1m tall
      0.0, -0.35, 0.3275,      // center at y=-0.35m (just behind -0.3)
      frame_id, "back_boundary"
    );
    RCLCPP_INFO(this->get_logger(), "Adding back boundary at y=-0.3m");

    // Side wall (x < -0.3) - wall to the side
    auto col_object_side = generateCollisionObject(
      0.1, 4.0, 1.0,           // size: 0.1m thick x 4m long x 1m tall
      -0.35, 0.0, 0.3275,      // center at x=-0.35m (just past -0.3)
      frame_id, "side_boundary"
    );
    RCLCPP_INFO(this->get_logger(), "Adding side boundary at x=-0.3m");

    // Ceiling boundary (z > 0.655) - ceiling above
    auto col_object_ceiling = generateCollisionObject(
      4.0, 4.0, 0.5,           // size: 4m x 4m x 0.5m thick
      0.0, 0.0, 0.655 + 0.25,  // center at z=0.905m
      frame_id, "ceiling_boundary"
    );
    RCLCPP_INFO(this->get_logger(), "Adding ceiling boundary at z=0.655m");

    // Apply all collision objects
    planning_scene_interface.applyCollisionObject(col_object_table);
    planning_scene_interface.applyCollisionObject(col_object_back);
    planning_scene_interface.applyCollisionObject(col_object_side);
    planning_scene_interface.applyCollisionObject(col_object_ceiling);

    RCLCPP_INFO(this->get_logger(), "Safety boundaries applied to planning scene");
    RCLCPP_INFO(this->get_logger(), "Robot will avoid:");
    RCLCPP_INFO(this->get_logger(), "  - Table: z < 0m");
    RCLCPP_INFO(this->get_logger(), "  - Back plane: y < -0.3m");
    RCLCPP_INFO(this->get_logger(), "  - Side plane: x < -0.3m");
    RCLCPP_INFO(this->get_logger(), "  - Ceiling: z > 0.655m");
  }

private:
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SafetyBoundaryNode>();

  RCLCPP_INFO(node->get_logger(), "Safety boundaries active. Press Ctrl+C to exit.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
