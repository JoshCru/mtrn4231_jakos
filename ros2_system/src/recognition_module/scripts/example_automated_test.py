#!/usr/bin/env python3
"""
Example: Automated Testing with Interactive Weight Simulation

This script demonstrates how to use the interactive weight simulation
programmatically for automated testing scenarios.

Note: This is a conceptual example. The actual interactive_weight_simulation
script is designed primarily for GUI interaction. For fully automated testing
without GUI, consider using the mock_camera_node or creating custom test fixtures.
"""

import numpy as np
import time


def generate_test_scenarios():
    """
    Generate various test scenarios for recognition module testing

    Returns:
        List of test scenarios, each containing object configurations
    """

    scenarios = []

    # Scenario 1: Objects in a line
    scenarios.append({
        'name': 'Linear Arrangement',
        'description': 'Objects arranged in a straight line',
        'objects': [
            {'weight': 50, 'pos': (0.2, 0.0, 0.1)},
            {'weight': 100, 'pos': (0.3, 0.0, 0.12)},
            {'weight': 200, 'pos': (0.4, 0.0, 0.115)},
        ]
    })

    # Scenario 2: Objects in corners
    scenarios.append({
        'name': 'Corner Placement',
        'description': 'Objects at workspace corners',
        'objects': [
            {'weight': 50, 'pos': (0.4, 0.4, 0.1)},
            {'weight': 100, 'pos': (0.4, -0.4, 0.12)},
            {'weight': 200, 'pos': (0.2, 0.4, 0.115)},
        ]
    })

    # Scenario 3: Clustered objects
    scenarios.append({
        'name': 'Clustered Arrangement',
        'description': 'Objects close together',
        'objects': [
            {'weight': 50, 'pos': (0.3, 0.0, 0.1)},
            {'weight': 100, 'pos': (0.32, 0.05, 0.12)},
            {'weight': 200, 'pos': (0.28, -0.05, 0.115)},
        ]
    })

    # Scenario 4: Grid pattern
    scenarios.append({
        'name': 'Grid Pattern',
        'description': 'Objects in 2x2 grid',
        'objects': [
            {'weight': 50, 'pos': (0.25, 0.1, 0.1)},
            {'weight': 100, 'pos': (0.25, -0.1, 0.12)},
            {'weight': 200, 'pos': (0.35, 0.1, 0.115)},
            {'weight': 150, 'pos': (0.35, -0.1, 0.11)},
        ]
    })

    # Scenario 5: Single object (baseline)
    scenarios.append({
        'name': 'Single Object',
        'description': 'Baseline test with one object',
        'objects': [
            {'weight': 100, 'pos': (0.3, 0.0, 0.12)},
        ]
    })

    return scenarios


def generate_point_cloud_for_object(weight_grams, position, add_noise=True):
    """
    Generate point cloud for a single weight object

    Args:
        weight_grams: Weight in grams (50, 100, 150, or 200)
        position: Tuple of (x, y, z) in meters
        add_noise: Whether to add Gaussian noise

    Returns:
        Numpy array of points with shape (N, 6) - [x, y, z, r, g, b]
    """

    # Object specifications (matching mock_camera_node)
    specs = {
        50:  {'width': 0.025, 'height': 0.025, 'depth': 0.01, 'color': (192, 192, 192)},
        100: {'width': 0.03,  'height': 0.03,  'depth': 0.015, 'color': (180, 180, 180)},
        150: {'width': 0.035, 'height': 0.035, 'depth': 0.012, 'color': (175, 175, 175)},
        200: {'width': 0.04,  'height': 0.04,  'depth': 0.015, 'color': (170, 170, 170)},
    }

    if weight_grams not in specs:
        raise ValueError(f"Invalid weight: {weight_grams}g")

    spec = specs[weight_grams]
    resolution = 0.002  # 2mm
    noise_stddev = 0.002 if add_noise else 0.0

    points = []

    # Generate points in cuboid
    dx, dy, dz = spec['width']/2, spec['height']/2, spec['depth']/2
    x0, y0, z0 = position

    x_range = np.arange(-dx, dx + resolution, resolution)
    y_range = np.arange(-dy, dy + resolution, resolution)
    z_range = np.arange(-dz, dz + resolution, resolution)

    for x in x_range:
        for y in y_range:
            for z in z_range:
                noise = np.random.normal(0, noise_stddev, 3) if add_noise else [0, 0, 0]

                point = [
                    x0 + x + noise[0],
                    y0 + y + noise[1],
                    z0 + z + noise[2],
                    spec['color'][0],
                    spec['color'][1],
                    spec['color'][2]
                ]
                points.append(point)

    return np.array(points)


def print_test_scenario(scenario, scenario_num):
    """Print test scenario details"""
    print(f"\n{'='*70}")
    print(f"Scenario {scenario_num}: {scenario['name']}")
    print(f"{'='*70}")
    print(f"Description: {scenario['description']}")
    print(f"Objects: {len(scenario['objects'])}")
    print()

    for i, obj in enumerate(scenario['objects'], 1):
        print(f"  Object {i}:")
        print(f"    Weight: {obj['weight']}g")
        print(f"    Position: ({obj['pos'][0]:.3f}, {obj['pos'][1]:.3f}, {obj['pos'][2]:.3f}) m")

    print()


def analyze_point_cloud(points):
    """Analyze generated point cloud"""
    print(f"Point Cloud Statistics:")
    print(f"  Total points: {len(points):,}")
    print(f"  X range: [{points[:,0].min():.3f}, {points[:,0].max():.3f}] m")
    print(f"  Y range: [{points[:,1].min():.3f}, {points[:,1].max():.3f}] m")
    print(f"  Z range: [{points[:,2].min():.3f}, {points[:,2].max():.3f}] m")
    print(f"  Centroid: ({points[:,0].mean():.3f}, {points[:,1].mean():.3f}, {points[:,2].mean():.3f}) m")


def main():
    """Main function for automated test generation"""

    print("="*70)
    print("Automated Weight Simulation Test Generator")
    print("="*70)
    print()
    print("This script generates test scenarios for the recognition module.")
    print("Use these scenarios with the interactive_weight_simulation.py script")
    print("or for automated testing.")
    print()

    # Generate scenarios
    scenarios = generate_test_scenarios()

    print(f"Generated {len(scenarios)} test scenarios:")
    for i, scenario in enumerate(scenarios, 1):
        print(f"  {i}. {scenario['name']}")
    print()

    # Process each scenario
    for i, scenario in enumerate(scenarios, 1):
        print_test_scenario(scenario, i)

        # Generate combined point cloud
        all_points = []
        for obj in scenario['objects']:
            points = generate_point_cloud_for_object(
                obj['weight'],
                obj['pos'],
                add_noise=True
            )
            all_points.append(points)

        combined_points = np.vstack(all_points)
        analyze_point_cloud(combined_points)

        print(f"\nTo test this scenario manually:")
        print(f"  1. Run: python3 interactive_weight_simulation.py --mode display --objects {len(scenario['objects'])}")
        print(f"  2. Position objects at the coordinates shown above")
        print(f"  3. Enable snap-to-grid (press S) for precise positioning")
        print()

        # Optional: Suggest ROS2 test command
        print(f"For ROS2 testing:")
        print(f"  python3 interactive_weight_simulation.py --mode publish --objects {len(scenario['objects'])}")
        print()

    # Summary
    print("="*70)
    print("Test Generation Complete")
    print("="*70)
    print()
    print("Next Steps:")
    print("  1. Run interactive_weight_simulation.py in display mode")
    print("  2. Manually configure objects according to test scenarios")
    print("  3. Switch to publish mode for recognition module testing")
    print("  4. Monitor /recognition/estimated_weights topic for results")
    print()
    print("Commands:")
    print("  # Display mode")
    print("  python3 interactive_weight_simulation.py --mode display --objects 3")
    print()
    print("  # Publish mode (requires ROS2)")
    print("  source install/setup.bash")
    print("  python3 interactive_weight_simulation.py --mode publish --objects 3")
    print()
    print("  # Monitor recognition output")
    print("  ros2 topic echo /recognition/estimated_weights")
    print()


if __name__ == '__main__':
    main()
