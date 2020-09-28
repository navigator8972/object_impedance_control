#plot position and orientation trajectory
#rxplot /kuka_allegro/obj_pose/pose/position/x:y:z /kuka_allegro/obj_pose/pose/orientation/w:x:y:z &
#rxplot /contactF1/fx:fy:fz &
rxplot /finger3/Pac[0] &
sleep 2
rosbag play $1
