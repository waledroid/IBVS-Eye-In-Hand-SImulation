# Visual servoing of robotic arm

This uses the Jacobian matrix to calculate velocity from the error vector: [u-u_star,v-v_star]. We didn’t find how to set the velocity, so we simulate the process still using target position control method. We assume each iter means one second in real world.

This also have 2 call_back functions,but the difference is here in the first call back function image_callback Only used the depth information,then used jacobian matrix and the 2d image error to calculate the relative position and rotation of aruco in each iteration(etc each second) and publish to the service '/movement' And need to mention that the reason we use service here because we think services are Synchronous. When your ROS program calls a service, your program can't continue until it receives a result from the service. The second callback function “target_callback”is almost the same with version one, it react to service request


#### Keywords
computer vision, visual servoing, xarm, ros, gazebo, python

