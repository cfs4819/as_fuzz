import os

def delete_non_error_videos(base_path):
    output_folders = [f for f in os.listdir(base_path) if f.startswith('output_') and os.path.isdir(os.path.join(base_path, f))]
    
    for output_folder in output_folders:
        camera_path = os.path.join(base_path, output_folder, 'camera')
        errors_path = os.path.join(base_path, output_folder, 'errors')
        
        if not os.path.exists(camera_path) or not os.path.exists(errors_path):
            continue
        
        try:
            error_files = [os.path.splitext(f)[0] for f in os.listdir(errors_path) if f.endswith('.json')]
        except FileNotFoundError:
            error_files = []
        
        try:
            for video_file in os.listdir(camera_path):
                video_name, video_ext = os.path.splitext(video_file)
                
                if video_name not in error_files:
                    video_path = os.path.join(camera_path, video_file)
                    print(f"Deleting {video_path}")
                    os.remove(video_path)
        except FileNotFoundError:
            continue

base_path = '/home/chenpansong/apollo_carla_8/apollo-r8.0.0/data/drivefuzz/24h_Town04/result_autorun_20240615_163126/'  
delete_non_error_videos(base_path)
