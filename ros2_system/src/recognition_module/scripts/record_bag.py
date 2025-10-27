#!/usr/bin/env python3
"""
ROS2 bag recording script for recognition and planning node testing
Provides convenient interface for recording test data
"""

import os
import sys
import argparse
import subprocess
from datetime import datetime
from pathlib import Path


class BagRecorder:
    """Helper class for recording ROS2 bags"""

    # Essential topics for recognition testing
    RECOGNITION_TOPICS = [
        '/camera/pointcloud',
        '/camera/image_raw',
        '/camera/depth/image_raw',
        '/camera/camera_info',
        '/camera/depth/camera_info',
        '/recognition/estimated_weights',
    ]

    # Planning-related topics (add when planning node is implemented)
    PLANNING_TOPICS = [
        '/planning/trajectory',
        '/planning/sorted_objects',
        '/robot/joint_states',
    ]

    # Transform topics
    TRANSFORM_TOPICS = [
        '/tf',
        '/tf_static',
    ]

    def __init__(self, bag_dir=None):
        if bag_dir is None:
            bag_dir = Path.home() / 'Documents' / 'mtrn4231_jakos' / 'test_bags'
        self.bag_dir = Path(bag_dir)
        self.bag_dir.mkdir(parents=True, exist_ok=True)

    def record(self, name=None, duration=None, topics='recognition', include_planning=False):
        """
        Record a ROS2 bag

        Args:
            name: Bag name (auto-generated if None)
            duration: Recording duration in seconds (None for manual stop)
            topics: 'recognition', 'planning', 'all', or list of topic names
            include_planning: Include planning topics
        """
        # Generate bag name
        if name is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            name = f"recognition_test_{timestamp}"

        bag_path = self.bag_dir / name

        # Select topics
        if isinstance(topics, str):
            topic_list = self._get_topic_list(topics, include_planning)
        else:
            topic_list = topics

        # Build command
        cmd = ['ros2', 'bag', 'record', '-o', str(bag_path)]
        cmd.extend(topic_list)

        if duration is not None:
            cmd.extend(['--max-duration', str(duration)])

        # Print info
        print("=" * 60)
        print("Recording ROS2 Bag for Recognition/Planning Testing")
        print("=" * 60)
        print(f"Bag name: {name}")
        print(f"Location: {bag_path}")
        print(f"\nRecording {len(topic_list)} topics:")
        for topic in topic_list:
            print(f"  - {topic}")

        if duration:
            print(f"\nDuration: {duration} seconds")
        else:
            print("\nPress Ctrl+C to stop recording")
        print("=" * 60)
        print()

        # Record
        try:
            subprocess.run(cmd, check=True)
            print()
            print("=" * 60)
            print("Recording complete!")
            print(f"Bag saved to: {bag_path}")
            print("=" * 60)
            return True
        except subprocess.CalledProcessError as e:
            print(f"\nError during recording: {e}", file=sys.stderr)
            return False
        except KeyboardInterrupt:
            print()
            print("=" * 60)
            print("Recording stopped by user")
            print(f"Bag saved to: {bag_path}")
            print("=" * 60)
            return True

    def _get_topic_list(self, topic_set, include_planning=False):
        """Get list of topics based on selection"""
        topics = []

        if topic_set in ['recognition', 'all']:
            topics.extend(self.RECOGNITION_TOPICS)

        if topic_set in ['planning', 'all'] or include_planning:
            topics.extend(self.PLANNING_TOPICS)

        if topic_set in ['all']:
            topics.extend(self.TRANSFORM_TOPICS)

        # Always include transforms for recognition
        if topic_set == 'recognition':
            topics.extend(self.TRANSFORM_TOPICS)

        return topics

    def list_bags(self):
        """List all recorded bags"""
        bags = sorted(self.bag_dir.glob('*'))

        if not bags:
            print(f"No bags found in {self.bag_dir}")
            return

        print(f"\nRecorded bags in {self.bag_dir}:")
        print("-" * 60)
        for bag in bags:
            if bag.is_dir():
                # Get bag info
                metadata_file = bag / 'metadata.yaml'
                if metadata_file.exists():
                    size = sum(f.stat().st_size for f in bag.rglob('*') if f.is_file())
                    size_mb = size / (1024 * 1024)
                    print(f"  {bag.name} ({size_mb:.2f} MB)")
                else:
                    print(f"  {bag.name}")
        print("-" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='Record ROS2 bags for recognition and planning testing'
    )

    subparsers = parser.add_subparsers(dest='command', help='Command')

    # Record command
    record_parser = subparsers.add_parser('record', help='Record a bag')
    record_parser.add_argument(
        '--name',
        type=str,
        default=None,
        help='Bag name (auto-generated if not specified)'
    )
    record_parser.add_argument(
        '--duration',
        type=int,
        default=None,
        help='Recording duration in seconds (Ctrl+C to stop if not specified)'
    )
    record_parser.add_argument(
        '--topics',
        type=str,
        choices=['recognition', 'planning', 'all'],
        default='recognition',
        help='Which topics to record'
    )
    record_parser.add_argument(
        '--include-planning',
        action='store_true',
        help='Include planning topics in recording'
    )
    record_parser.add_argument(
        '--bag-dir',
        type=str,
        default=None,
        help='Directory to store bags'
    )

    # List command
    list_parser = subparsers.add_parser('list', help='List recorded bags')
    list_parser.add_argument(
        '--bag-dir',
        type=str,
        default=None,
        help='Directory containing bags'
    )

    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    recorder = BagRecorder(bag_dir=args.bag_dir if hasattr(args, 'bag_dir') else None)

    if args.command == 'record':
        recorder.record(
            name=args.name,
            duration=args.duration,
            topics=args.topics,
            include_planning=args.include_planning
        )
    elif args.command == 'list':
        recorder.list_bags()


if __name__ == '__main__':
    main()
