# AutonomX_state_estimator















# ---------------------------------------------------------------------------------------- #
# Encoder #


day 11/3: 
	-> added rosgraph_msg to package.xml
	-> subscribed to /clock. "time clock" clock variable of type time (seconds & nseconds)
	
	-> changed time to simulation time in cuboid:
	simulationTime = sim.getSimulationTime()
    	simROS.publish(pub_clock,{clock=simulationTime})

day 12/3:
	v1:
	-> added an odom_subscriber, initialized values of position x, y, z"
    	
    	
> 

