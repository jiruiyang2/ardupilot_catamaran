from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time

# Connect to SITL
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Clear existing mission
cmds = vehicle.commands
cmds.clear()
cmds.wait_ready()

# Define a lawnmower grid mission
home = vehicle.location.global_relative_frame
waypoints = [
    LocationGlobalRelative(home.lat + 0.00005, home.lon, 0),
    LocationGlobalRelative(home.lat + 0.00005, home.lon + 0.0001, 0),
    LocationGlobalRelative(home.lat + 0.00010, home.lon + 0.0001, 0),
    LocationGlobalRelative(home.lat + 0.00010, home.lon, 0),
]

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                 home.lat, home.lon, 0))  # Home point

for wp in waypoints:
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                     wp.lat, wp.lon, wp.alt))

cmds.upload()
print("Mission uploaded.")

# Start mission
vehicle.mode = VehicleMode("AUTO")
while not vehicle.mode.name == 'AUTO':
    time.sleep(0.5)

vehicle.armed = True
while not vehicle.armed:
    time.sleep(0.5)

print("Vehicle armed and mission started.")

while vehicle.commands.next < len(waypoints) + 1:
    print(f"Currently at waypoint {vehicle.commands.next}")
    time.sleep(2)

print("Mission complete.")
vehicle.mode = VehicleMode("HOLD")
vehicle.close()
