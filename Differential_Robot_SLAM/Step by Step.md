#### Odometry Calibration
Odometry is widely used method to estimate actual position relative to starting position. The output will use in EKF Localization simultaneously with IMU or various sensor (Sensor Fusion) to make Odometry more accurate. So we need to make sure if wheel odometry resulting correct value. 
 
There are two source error in odometry:
<p align="center">
    <img src="/images/odometry_error.png"
         alt="Odometry Error"
         width="90%" />
</p>

**1. Systematic Error**
- Unequal wheel diameters
- Wrong kinematic equation
- Average of both wheel diameters differs from nominal diameter
- Misalignment of wheels
- Uncertainty about the effective wheelbase (due to non-point wheel contact with the floor)
- Limited encoder resolution
- Limited encoder sampling rate

**2. Non Systematic Error**
- uneven floor
- Travel over uneven floors or Travel over unexpected objects on the floor
- Wheel-slippage due to:
• slippery floors
• over-acceleration
• fast turning (skidding)
• external forces (interaction with external bodies)
• internal forces (e.g., castor wheels)
• non-point wheel contact with the floor
  
Only Systematic error can solve with Calibration, We need to add algorithm to solve non systematic error (EKF etc).

**UMBmark** (*University of Michigan Benchmark*) is Calibration methode using Bidirectional square path to calibrate odometry caused systematic errors. For Improving last methode using Uni-directional Square Path that have disadvantages because one of these two caused error can have same final position.

step by step to calibrate odometry:
1. Define Nominal Parameter:
    - Wheel diameter nominal \(D_n\)
    - Wheelbase nominal \(b_n\)
    - Encoder resolution
    - Gear ratio
    - Encoder counts
2. Create Square Path 4x4m
3. Define Initial Position
```bash
Odometry:
x = 0
y = 0
theta = 0 
```
4. Move the Robot thought the square path 4x4m in Clockwise direction and make sure:
   - stop after each 4 m straight leg
   - make a total of four 90 -turns on the spot
   - run the vehicle slowly to avoid slippage.
Run only using Kinematic equation without Path Controller to make sure the robot follow the path.

5. Measure the Final Position to get return position error
6. Repeaat this step 3-5 in 5 times in CW direction and CCW Direction.
7. Calculate Center of gravitu in Both of Direction
8. Calculate UMBmark Error
9. Separate of Type A Error and Type B Error
10. Calculate Eb Wheelbase correction and Ed wheel diameter correction.
    
UMBmark is primarily designed to identify and compensate the two dominant systematic odometry errors in differential-drive mobile robots: unequal wheel diameters and uncertainty in the effective wheelbase. Other systematic errors, such as wheel misalignment, encoder resolution, and encoder sampling limitations, may contribute to the measured Type A and Type B errors but are not independently identified by the standard UMBmark procedure

**Reference**

* J. Borenstein and L. Feng, *Measurement and Correction of Systematic Odometry Errors in Mobile Robots*, IEEE Transactions on Robotics and Automation, Vol. 12, No. 6, 1996, pp. 869-880.
  [Paper PDF](https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf)

* Changbae Jung and Woojin Chung, *Design of Test Tracks for Odometry Calibration of Wheeled Mobile Robots*, Adv Robotic Sy, 2011, Vol. 8, No. 4, 1-9.
  [Paper PDF](https://www.researchgate.net/publication/221915426_Design_of_Test_Tracks_for_Odometry_Calibration_of_Wheeled_Mobile_Robots)