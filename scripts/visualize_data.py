import rerun as rr
import time

def main():
    print("Launching Rerun viewer to connect to the data stream...")
    print("Run the teleoperation and data recording nodes in a separate terminal.")
    
    # This will connect to an existing Rerun server spawned by the data_recorder
    # or spawn a new one if none is found.
    rr.init("so101_teleop_data", spawn=False)
    
    print("Viewer should be connected. Waiting for data...")
    print("Press Ctrl+C to exit.")

    try:
        # Keep the script alive to keep the viewer connection open
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting visualizer.")

if __name__ == '__main__':
    main() 