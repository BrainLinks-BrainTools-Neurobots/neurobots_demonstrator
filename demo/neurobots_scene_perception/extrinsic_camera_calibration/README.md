
---------------------------------------------------

Script needs to be executed on computer "cube1"
 
---------------------------------------------------

Syntax:

rosrun neurobots_launch start_extrinsic_calibration camera_name sync_queue_size

---------------------------------------------------

For calibration of the shelf camera:

roslaunch neurobots_launch extrinsic_calibration.launch camera:=shelf
camera_name = shelf
sync_queue_size = 5

---------------------------------------------------

For calibration of the table camera:

camera_name = table
sync_queue_size = 15

---------------------------------------------------

Remarks:

"sync_queue_size" is related to the delay between the computer which runs the camera and the computer that runs Simtrack.

If nothing is published on topic "simtrack/image" (simtrack tracker output) you probably need to increase the value for "sync_queue_size"

---------------------------------------------------
