# State Estimator Package

## Package Summary

The **autonomx_state_estimator** package contains nodes related to vehicle state estimation. The package contains two ROS nodes as follows:
    
* `state_estimator_butterworth_kf`: An earlier attempt at improving the estimator's peformance by using a butterworth low-pass filter before the main KF. However, this proved inefficent and can thus be safely ignored.

* `state_estimator_ms`: The main state estimator used in this work, based on the Kalman Filter, to estimate the vehicle's pose. The process model is based on the kinematic bicycle model, while the sensor model is represented by an identity matrix as the sensor observations are the same as the estimator's states.  

As well as two scripts which were used to analyze the results of the estimation, collected using ROS Bags, and to validate the accuracy of the vehicle model respectively. 

## Nodes Summary

### State Estimator (KF)

#### Node Parameters:
* `node`: Determines which of the two nodes is launched (should be kept to the default value).
* `odometry_topic_name`: Name of the topic where the vehicle's current pose and twist are published.
* `steering_topic_name`: Name of the topic to publish calculated steering angles to, according to the vehicle model within the simulator.
* `velocity_topic_name`: Name of the topic to which the target velocity in (m/s) is published.
* `wheelbase`: The length of the vehicle's wheelbase in meters.