# Detection-SARWAI

The visual detection modules requires darknet libraries. The ROS wrapper we use for darknet requires a specific commit. These commands will get the submodule and checkout the appropriate commit for compilation.

`git submodule init`
`git submodule update`
`cd src/darknet_ros/darknet`
`git checkout 59ed171`
