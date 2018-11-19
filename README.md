# matlab_xyyaw_correlation
Demonstrates how a measurement of x,y allows us to estimate yaw as long as our motion model introduces a correlation between yaw and x,y

Can be used with Matlab or Octave

We assume a measurement model where we measure x,y directly. We can set the uncertainty in these measurements with the variables xMeasStd and yMeasStd.

The scrip allows you to test to types of motion models and combinations thereof. In the first one, we command the robot with speeds in the body frame vx, vy, yawrate. With these commands the new position will depend on the yaw angle, thus creating a correlation between x,y and yaw. This means that when we measure x,y we will gain information about yaw and thus with time find also yaw even though we do not measure it directly. Note that if we do not move there is no correlation and thus we will not be able to estimate x,y. In the second motion model we let vx=vy=0 and yawrate=0 and instead let the pose be driven by noise. This would be a very crude model to for example represent the fact that we really have no clue about what causes the motion, such as when someone is holding the robot and moving it around and. It is crdue becuase normally there would be some inertia and thus a constant velocity model might be more realistic. That is, we include another state representing the speed of the platform and instead add the noise there, corresponding to the acceletation.

If you let the parameter displayCorrelations be set to 1 the simulation will display two images. Make sure to arrange them such that they are not ontop of each other before you proceed.

## Estimate yaw from x,y measurements
Make sure that driveByPureNoise = 0 in EKF.m (line 75). This will command the robot to drive in a circle with radius 5m. Note that we can do this without knowing where we are. We just do not know where the circle will end up in the world. Run EKF and you will see how the position is found quite quickly and eventually also yaw. Once the filter has converged you will see that the yaw angle is well estimated all the time and that x and y will alternate to have the largest uncertainrty depending on the current heading. 

## Not able to estimate yaw from x,y measurements
Make sure that driveByPureNoise = 1 in EKF.m (line 46). This will switch to a model where the motion is driven entirely by noise. In this case, no correlation is created byween x,y and yaw and thus measuring x,y says nothing about yaw and it will not be estimated.
