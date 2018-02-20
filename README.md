# Camera SignalMan

> A nodelet to dynamically reroute cameras image feed topics.

## Dependencies

* [elikos_msgs](https://github.com/elikos/elikos_msgs)

## Running SignalMan 

Start video streams 
```
roslaunch video_stream_opencv webcam.launch
roslaunch video_stream_opencv webcam2.launch
```
Run SignalMan
```
roslaunch camera_signalman camera_signalman_nodelet.launch
```
view output via image_view 
```
rosrun image_view image_view image:=/camersignalman/image_raw
```
