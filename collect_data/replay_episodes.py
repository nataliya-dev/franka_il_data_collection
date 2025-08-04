import os
import h5py
import argparse
import numpy as np
import cv2
import time
import glob
from franka_robot import FrankaRobot


def list_episode_files(data_directory):
    """List all episode files in the data directory"""
    pattern = os.path.join(data_directory, "episode_*.hdf5")
    episode_files = glob.glob(pattern)
    episode_files.sort()  # Sort files chronologically
    return episode_files


def select_episode(episode_files):
    """Display available episodes and let user select one"""
    if not episode_files:
        print("No episode files found in the directory!")
        return None

    print("\nAvailable episodes:")
    print("-" * 50)
    for idx, file_path in enumerate(episode_files):
        filename = os.path.basename(file_path)
        # Extract timestamp from filename for better display
        timestamp = filename.replace("episode_", "").replace(".hdf5", "")
        print(f"{idx + 1:2d}. {filename} ({timestamp})")

    print("-" * 50)

    while True:
        try:
            choice = input(
                f"\nSelect episode (1-{len(episode_files)}) or 'q' to quit: ").strip()

            if choice.lower() == 'q':
                return None

            choice_idx = int(choice) - 1

            if 0 <= choice_idx < len(episode_files):
                selected_file = episode_files[choice_idx]
                print(f"\nSelected: {os.path.basename(selected_file)}")
                return selected_file
            else:
                print(
                    f"Please enter a number between 1 and {len(episode_files)}")

        except ValueError:
            print("Please enter a valid number or 'q' to quit")


def main(args):
    # Default data directory - you can modify this path
    data_directory = args.get('data_dir')

    if not os.path.isdir(data_directory):
        print(f'Data directory does not exist at \n{data_directory}\n')
        exit()

    # List and select episode
    episode_files = list_episode_files(data_directory)
    dataset_path = select_episode(episode_files)

    if dataset_path is None:
        print("No episode selected. Exiting.")
        return

    if not os.path.isfile(dataset_path):
        print(f'Selected dataset does not exist at \n{dataset_path}\n')
        exit()
    else:
        print(f'Loading dataset: \n{dataset_path}\n')

    robot = FrankaRobot()
    robot.move_home()

    with h5py.File(dataset_path, 'r') as root:
        actions = root['/action'][()]
        tm = root['/tm'][()]
        camera_images1 = root['/observations/images/wrist'][()]
        camera_images2 = root['/observations/images/ext1'][()]
        camera_images3 = root['/observations/images/ext2'][()]
        qpos = root['/observations/qpos'][()]

        print(f"Episode contains {len(actions)} actions")
        print("Press 'q' during image display to skip to robot execution")

        # Display images
        for idx, img in enumerate(camera_images1):
            images = np.hstack((img, camera_images2[idx], camera_images3[idx]))
            color_image = np.asanyarray(images)
            cv2.imshow("Camera Images", color_image)
            time.sleep(0.05)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

        # Execute robot actions
        print("Starting robot execution...")
        for idx, action in enumerate(actions):
            if idx == len(actions) - 1:
                dt = [0.1]
            else:
                dt = tm[idx + 1]

            robot.move_velocity_array(
                linear_velocity=action[0:3],
                angular_velocity=action[3:6],
                gripper=action[6]
            )
            print(f"Action {idx + 1}/{len(actions)}, dt: {dt}")
            time.sleep(float(dt[0]))

        time.sleep(1)

        expected_position = qpos[-1]
        final_position = robot.robot.current_joint_positions
        print(f"{expected_position=}")
        print(f"{final_position=}")
        print(f"diff = {expected_position-final_position}")

        robot.move_home()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Replay robot episodes interactively')
    parser.add_argument('--data_dir', action='store', type=str,
                        default="data",
                        help='Directory containing episode files')

    main(vars(parser.parse_args()))
