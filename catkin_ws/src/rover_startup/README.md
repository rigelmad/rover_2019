# How to Start the Rover

With the help of these launch files, starting up the rover is a ~1 min process

### Hardware Setup
**Note:** This assumes all USB cams, FTDI cables, batteries are all set up on the rover and turned on.
1. Obtain the following:
 - Base Station computer
 - USB Hub (unpowered is fine)
 - Radio package (includes the Ubiquitous Rocket, the battery pack, & the POE adapter - should all be kept together already)
 - Two Xbox controllers (or just one if all you want to do is drive)
2. Plug Ethernet _directly into computer_ (the hub is more susceptible to being disconnected from random movement), and turn on the battery pack so that the green LED turns on
3. Plug in the hub to the remaining USB port, and plug the Xbox controller(s) in
4. Get comfy

### Software Setup
1. Open two terminal windows side by side. These will be henceforth referred to as **Window 1** and **Window 2**
2. On **Window 1**, execute the following commands:
  - `ssh nvidia@192.168.1.2` - SSH into the Jetson, assuming hardware is setup correctly. I'll write debug steps later
  - This step will vary based on what systems you are trying to set up:
    - Drive: `roslaunch rover_startup rover_startup_drive.launch`
    - Arm & Drive: `roslaunch rover_startup rover_startup_arm.launch`
    - LD & Drive: `roslaunch rover_startup rover_startup_ld.launch`
  - Wait ~10s or so for things to set up, then move on to next step
3. On **Window 2**, execute the following commands:
  - `muri` - Alias to setup up ROS environment variables to look at the Jetson for Roscore
  - This step will vary based on what systems you are trying to set up (these autocomplete):
    - Drive: `roslaunch rover_startup base_station_startup_drive.launch`
    - Arm & Drive: `roslaunch rover_startup base_station_startup_arm.launch`
    - LD & Drive: `roslaunch rover_startup base_station_startup_ld.launch`
