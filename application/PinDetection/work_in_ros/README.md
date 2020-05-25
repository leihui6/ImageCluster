# This is a package of ros.

## test
please follow these command to test this package
```shell
$roscore
$rosrun pin_detection_pkg pin_detection_pkg_publisher_node xxx.jpg
$rosrun pin_detection_pkg pin_detection_pkg_subscriber_node
```

## Message type

from `msg/pin_detection_result.msg`:

```
std_msgs/Bool is_has_needle
geometry_msgs/Point opening_position
geometry_msgs/Point closing_position

geometry_msgs/Point rotate_direction_begin
geometry_msgs/Point rotate_direction_end

geometry_msgs/Point ceter_position
```
