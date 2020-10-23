
# Rover_mini

This repo contains all necessary packages specific to the rover_mini mobile base.

## rover_mini_control
In this package, you can find the base_controller node, which controls the motors, reads the encoders and sends the encoders values while taking wrap around into account. 

To launch it use `roslaunch rover_mini_control base_controller.launch`

To launch the base controller along with the differential drive, use `roslaunch rover_mini_control base_controller_with_diff_drive.launch` *Note: you will need the differential_drive package for this to work
