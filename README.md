# ros1bag_to_pcd

## Environment
ROS1

## Build
```
cd catkin_ws/src
git clone https://github.com/KOKIAOKI/ros1bag_to_pcd.git
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Run
### Offline
You can convert "sensor_msgs/PointCloud2" msgs to pcd files.
```
cd catkin_ws
source devel/setup.bash
rosrun ros1bag_to_pcd offline [your bag file path] [topic name] [output_folder path]
```

or

```
./devel/lib/ros1bag_to_pcd/offline [your bag file path] [topic name] [output_folder path]
```