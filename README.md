# Drone-Orientation-Filters

The commands for each filter are straightforward:

`rosrun drone-orientation-filters complimentary-filter`

`rosrun drone-orientation-filters kalman-filter`

`rosrun drone-orientation-filters extended-kalman-filter`

There is also a node for converting from a MAVROS pose message to the euler angles for comparison:

`rosrun drone-orientation-filters rpy-truth`
