import os
import json
import pandas as pd
import imageio.v2 as imageio
import argparse
from pathlib import Path
import shutil

def convert_episode(raw_episode_path, lerobot_dataset_path):
    """
    Converts a single raw episode directory to a LeRobot dataset episode.

    The raw episode directory should contain:
    - images/ (folder of .png images)
    - joint_states.jsonl
    - controller_poses.jsonl
    """
    print(f"Processing episode: {raw_episode_path}")
    raw_episode_path = Path(raw_episode_path)
    episode_name = raw_episode_path.name
    lerobot_episode_path = Path(lerobot_dataset_path) / episode_name
    
    if lerobot_episode_path.exists():
        print(f"Episode already exists in LeRobot dataset. Skipping: {episode_name}")
        return

    # Create LeRobot episode directory structure
    lerobot_episode_path.mkdir(parents=True, exist_ok=True)
    (lerobot_episode_path / 'videos').mkdir(exist_ok=True)

    # 1. Convert images to video
    print("Converting images to video...")
    image_files = sorted(raw_episode_path.glob('images/*.png'))
    if not image_files:
        print(f"No images found in {raw_episode_path}. Skipping episode.")
        return
        
    video_path = lerobot_episode_path / 'videos' / 'episode.mp4'
    with imageio.get_writer(video_path, fps=30) as writer:
        for img_file in image_files:
            writer.append_data(imageio.imread(img_file))

    # 2. Process metadata into a parquet file
    print("Processing metadata...")
    joint_states_path = raw_episode_path / 'joint_states.jsonl'
    poses_path = raw_episode_path / 'controller_poses.jsonl'

    if not joint_states_path.exists() or not poses_path.exists():
        print(f"Missing data files in {raw_episode_path}. Skipping.")
        return

    with open(joint_states_path, 'r') as f:
        joint_data = [json.loads(line) for line in f]
    
    with open(poses_path, 'r') as f:
        pose_data = [json.loads(line) for line in f]

    # This is a simplification. A real implementation would need to carefully
    # align timestamps and structure the data exactly as LeRobot expects.
    # For now, we create a simple combined dataframe.
    df_joints = pd.DataFrame(joint_data)
    df_poses = pd.DataFrame(pose_data)
    
    # Example of merging - needs careful alignment in a real scenario
    df = pd.concat([df_joints.add_prefix('joints.'), df_poses.add_prefix('poses.')], axis=1)
    
    # Add frame numbers to link with video
    df['frame_id'] = range(len(df))
    df['timestamp'] = df['joints.timestamp'] # Use one of the timestamps

    # Save to parquet
    parquet_path = lerobot_episode_path / 'hf_dataset.parquet'
    df.to_parquet(parquet_path)

    # 3. Create metadata.json
    print("Creating metadata.json...")
    num_frames = len(image_files)
    metadata = {
        "episode_index": int(episode_name.split('_')[1]), # Assumes name is 'episode_...'
        "timestamps": df['timestamp'].tolist(),
        "frame_rate": 30,
        "video_path": str(video_path.relative_to(lerobot_episode_path)),
        "parquet_path": str(parquet_path.relative_to(lerobot_episode_path)),
        "total_frames": num_frames,
    }
    with open(lerobot_episode_path / 'metadata.json', 'w') as f:
        json.dump(metadata, f, indent=4)
        
    print(f"Successfully converted episode {episode_name}")

def main():
    parser = argparse.ArgumentParser(description="Convert raw recorded data to LeRobot dataset format.")
    parser.add_argument(
        "--input_dir",
        type=str,
        default=os.path.expanduser("~/data/raw_episodes"),
        help="Directory containing the raw episode recordings.",
    )
    parser.add_argument(
        "--output_dir",
        type=str,
        default=os.path.expanduser("~/data/lerobot_episodes"),
        help="Directory to save the LeRobot dataset.",
    )
    args = parser.parse_args()

    Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    
    for episode_dir in sorted(Path(args.input_dir).iterdir()):
        if episode_dir.is_dir():
            convert_episode(episode_dir, args.output_dir)
            
    print("\nConversion process finished.")

if __name__ == "__main__":
    main() 