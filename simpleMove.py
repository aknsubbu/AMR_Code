#!/usr/bin/env python3
"""
Enhanced Beacon Tracker with Calibration-Based and Manual Movement
- Select beacon and calibrate
- Option to perform movements based on calibration results
- Press Enter to move forward at speed 128
- Press Enter again to stop
"""
import asyncio
import logging
import sys
from beaconTrackerController import BeaconTrackerController
from modules.motion_controller.motion_controller import MotionController
from modules.motion_controller.serial_communicator import SerialCommunicator

# Set up logging
logging.basicConfig(
    level=logging.INFO,  # Changed to INFO for less verbose output
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'  # Arduino serial port
TARGET_BEACON = None          # Set to None for selection prompt
DEMO_MODE = False             # Set to False for real hardware

# Tracking parameters
SCAN_INTERVAL = 0.5           # Beacon scan interval
RSSI_SAMPLES = 3              # RSSI samples to average
RSSI_THRESHOLD = -80          # Signal threshold
MOVEMENT_SPEED = 225          # Standard movement speed (as requested)

async def perform_suggested_movement(motion_controller, best_direction, best_angle, duration=2.0):
    """
    Perform movement based on calibration results
    """
    # First turn to the best angle
    turn_time = abs(best_angle) / 90.0  # Assuming 90 degrees takes 1 second
    
    print(f"🔄 Turning to {best_angle}° for {turn_time:.1f} seconds...")
    
    if 0 <= best_angle < 180:
        # Turn right
        motion_controller.turn_right(MOVEMENT_SPEED)
    else:
        # Turn left
        motion_controller.turn_left(MOVEMENT_SPEED)
    
    await asyncio.sleep(turn_time)
    motion_controller.stop()
    await asyncio.sleep(0.5)  # Short pause after turning
    
    # Then move in the best direction
    print(f"➡️ Moving {best_direction.lower()} for {duration} seconds...")
    
    if best_direction == "FORWARD":
        motion_controller.forward(MOVEMENT_SPEED)
    elif best_direction == "BACKWARD":
        motion_controller.backward(MOVEMENT_SPEED)
    elif best_direction == "LEFT":
        motion_controller.turn_left(MOVEMENT_SPEED)
    elif best_direction == "RIGHT":
        motion_controller.turn_right(MOVEMENT_SPEED)
    
    await asyncio.sleep(duration)
    motion_controller.stop()
    
    print("✅ Suggested movement complete")
    return True

async def main():
    # Create event bus
    class EventBus:
        def __init__(self):
            self.handlers = {}
        
        def register(self, event, handler):
            if event not in self.handlers:
                self.handlers[event] = []
            self.handlers[event].append(handler)
        
        def publish(self, event, data=None):
            if event in self.handlers:
                for handler in self.handlers[event]:
                    handler(data)
    
    event_bus = EventBus()
    
    # Initialize components
    serial_comm = SerialCommunicator(SERIAL_PORT, event_bus, demo_mode=DEMO_MODE)
    motion_controller = MotionController(serial_comm, event_bus)
    
    # Create the beacon tracker
    tracker = BeaconTrackerController(
        motion_controller=motion_controller,
        target_beacon_name=TARGET_BEACON,
        scan_interval=SCAN_INTERVAL,
        rssi_samples=RSSI_SAMPLES,
        rssi_threshold=RSSI_THRESHOLD,
        movement_speed=MOVEMENT_SPEED
    )
    
    # Start the tracker
    await tracker.start()
    
    try:
        # Test basic movement to ensure hardware is working
        print("\n🔍 Testing basic movement...")
        motion_controller.forward(MOVEMENT_SPEED)
        await asyncio.sleep(0.5)
        motion_controller.stop()
        motion_controller.turn_right(MOVEMENT_SPEED)
        await asyncio.sleep(0.5)
        motion_controller.stop()
        print("✅ Basic movement test complete")
        
        # Scan for available beacons
        print("\n🔍 Scanning for BLE beacons...")
        beacons = await tracker.scan_for_beacons(duration=5.0)
        
        if not beacons:
            print("❌ No beacons found. Exiting.")
            return
        
        # Print found beacons with RSSI
        print(f"\n📡 Found {len(beacons)} beacons:")
        sorted_beacons = sorted(beacons.items(), key=lambda x: x[1]['rssi'], reverse=True)
        for i, (name, data) in enumerate(sorted_beacons, 1):
            rssi = data['rssi']
            print(f"  {i}. {name}: RSSI {rssi} dBm {'(STRONG)' if rssi > -60 else ''}")
        
        # Simple single-step beacon selection
        print("\n👉 Enter the number of the beacon to calibrate with (or press Enter to quit):")
        selection = input("> ")
        
        if not selection.strip():
            print("Exiting...")
            return
        
        try:
            index = int(selection) - 1
            if 0 <= index < len(sorted_beacons):
                beacon_name = sorted_beacons[index][0]
                tracker.set_target_beacon(beacon_name)
                print(f"\n✅ Selected beacon: {beacon_name}")
                
                # Calibrate with the selected beacon
                print("\n⚙️ Calibrating with selected beacon...")
                
                def calibration_progress(message):
                    print(f"  > {message}")
                
                calibration_result = await tracker.calibrate(callback=calibration_progress)
                
                if calibration_result:
                    best_direction = calibration_result.get('best_direction')
                    print(f"\n✅ Calibration complete. Best direction: {best_direction}")
                    
                    best_angle = 0
                    if 'rotation_map' in calibration_result:
                        best_angle = max(calibration_result['rotation_map'].items(), key=lambda x: x[1])[0]
                        print(f"  Best angle: {best_angle}°")
                        print(f"  Peak signal: {calibration_result['signal_peak']} dBm")
                    
                    # Offer to execute the suggested movement
                    print(f"\n⚠️ Calibration suggests: {best_direction} at {best_angle}° angle")
                    print("Would you like to perform this movement? (y/n)")
                    execute_suggestion = input("> ")
                    
                    if execute_suggestion.lower() == 'y':
                        await perform_suggested_movement(motion_controller, best_direction, best_angle)
                    
                    # Simple manual control loop - just press Enter to move or stop
                    print("\n🔄 Simple Manual Control:")
                    print("  - Press ENTER to move forward at speed 128")
                    print("  - Press ENTER again to stop")
                    print("  - Type 's' to perform the suggested movement again")
                    print("  - Type 'q' and press ENTER to quit")
                    
                    is_moving = False
                    
                    while True:
                        # Display current state
                        state = tracker.get_state()
                        status = "MOVING" if is_moving else "STOPPED"
                        rssi = state['rssi'] if state['rssi'] is not None else "N/A"
                        
                        # Prompt for input
                        user_input = input(f"\n📡 Status: {status}, RSSI: {rssi} dBm | Press ENTER to toggle movement, 's' for suggested movement, or 'q' to quit > ")
                        
                        if user_input.lower() == 'q':
                            print("🛑 Quitting...")
                            break
                        elif user_input.lower() == 's':
                            # Stop any current movement first
                            if is_moving:
                                motion_controller.stop()
                                is_moving = False
                            
                            # Perform suggested movement
                            await perform_suggested_movement(motion_controller, best_direction, best_angle)
                        elif user_input.lower()=='l2':
                            motion_controller.turn_left(128)

                        elif user_input.lower()=='r2':
                            motion_controller.turn_right(128)
                        else:
                            # Toggle movement state
                            if is_moving:
                                # Stop the robot
                                motion_controller.stop()
                                print("🛑 Stopped")
                                is_moving = False
                            else:
                                # Move forward at speed 128
                                motion_controller.forward(150)  # Always use exactly 128 as requested
                                print("➡️ Moving forward at speed 225")
                                is_moving = True
                else:
                    print("❌ Calibration failed. Please check if the beacon is in range.")
            else:
                print("❌ Invalid selection. Exiting.")
        except ValueError:
            print("❌ Invalid input. Exiting.")
        
    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
    finally:
        # Stop the tracker and clean up
        motion_controller.stop()  # Ensure the robot stops
        await tracker.stop()
        if serial_comm:
            serial_comm.close()
        print("\n✅ Cleanup complete")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n🛑 Program terminated by user")