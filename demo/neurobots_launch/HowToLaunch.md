------------------------ Calibration ----------------------

1) +++ MAP to SHELF Camera Calibration

’roslaunch neurobots_launch hardware.launch
roslaunch neurobots_launch demo_calibration.launch camera:=shelf
roslaunch neurobots_calibration start_map_to_shelf_calibration.launch’

Extrinsic Parameters stored in: neurobots_calibration/calibration_files/map_to_shelf_camera.mat

2) +++ MAP to TABLE Camera Calibration

’roslaunch neurobots_launch hardware.launch
roslaunch neurobots_launch demo_calibration.launch camera:=table
roslaunch neurobots_calibration start_map_to_table_calibration.launch’

Extrinsic Parameters stored in: neurobots_calibration/calibration_files/map_to_table_camera.mat

2) +++ TABLE Camera to IIWA base_link Calibration

’roslaunch neurobots_calibration start_iiwa_to_camera_calibration.launch’

Extrinsic Parameters stored in: neurobots_calibration/calibration_files/table_camera_to_iiwa.mat

