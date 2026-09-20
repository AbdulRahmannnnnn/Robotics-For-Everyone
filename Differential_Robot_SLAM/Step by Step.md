#### Odometry Calibration
Odometry is widely used method to estimate actual position relative to starting position. The output will use in EKF Localization simultaneously with IMU or various sensor (Sensor Fusion) to make Odometry more accurate. So we need to make sure if wheel odometry resulting correct value. 
 
There are two source error in odometry:

**1. Systematic Error**
- Wrong Wheel radius value
- Wrong Wheel_seperation value
- Different Wheel radius for left and right wheel
- Wrong kinematic equation

**2. Non Systematic Error**
- Wheel slippage
- uneven floor
- Disturbance from External(there are something push the robot)
  
Only Systematic error can solve with Calibration, We need to add algorithm to solve non systematic error (EKF etc).

step by step to calibrate odometry: