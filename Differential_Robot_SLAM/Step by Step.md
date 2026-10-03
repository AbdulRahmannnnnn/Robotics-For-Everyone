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

<p align="center">
    <img src="/images/UMBmark.png"
         alt="UMBmark"
         width="80%" />
</p>

**Reference**

* J. Borenstein and L. Feng, *Measurement and Correction of Systematic Odometry Errors in Mobile Robots*, IEEE Transactions on Robotics and Automation, Vol. 12, No. 6, 1996, pp. 869-880.
  [Paper PDF](https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf)

* Changbae Jung and Woojin Chung, *Design of Test Tracks for Odometry Calibration of Wheeled Mobile Robots*, Adv Robotic Sy, 2011, Vol. 8, No. 4, 1-9.
  [Paper PDF](https://www.researchgate.net/publication/221915426_Design_of_Test_Tracks_for_Odometry_Calibration_of_Wheeled_Mobile_Robots)


#### Extended Kalman Filter Localization Calibration

# Differential Drive Motion Model

Motion model digunakan oleh Extended Kalman Filter (EKF) untuk memprediksi perubahan posisi robot berdasarkan keadaan robot sebelumnya dan input gerak.

Untuk robot differential drive, state robot dapat disederhanakan menjadi:

$$
\mathbf{x} =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
$$

dengan:

* $x$ : posisi robot pada sumbu $X$ [m]
* $y$ : posisi robot pada sumbu $Y$ [m]
* $\theta$ : orientasi robot terhadap sumbu $X$ [rad]

Input gerak robot adalah:

$$
\mathbf{u} =
\begin{bmatrix}
v \\
\omega
\end{bmatrix}
$$

dengan:

* $v$ : kecepatan linear robot [m/s]
* $\omega$ : kecepatan angular robot [rad/s]

## 1. Continuous Motion Model

Untuk differential-drive robot, model gerak dapat dituliskan sebagai:

$$
\dot{x} = v\cos(\theta)
$$

$$
\dot{y} = v\sin(\theta)
$$

$$
\dot{\theta} = \omega
$$

atau dalam bentuk matriks:

$$
\dot{\mathbf{x}} =
\begin{bmatrix}
v\cos(\theta) \\
v\sin(\theta) \\
\omega
\end{bmatrix}
$$

Model ini menunjukkan bahwa kecepatan linear $v$ bekerja sepanjang arah heading robot.

---

## 2. Discrete Motion Model

Karena EKF bekerja secara diskrit, model tersebut dapat digunakan untuk menghitung keadaan robot pada waktu berikutnya.

Dengan timestep $\Delta t$:

$$
x_{k+1}
=
x_k + v_k\cos(\theta_k)\Delta t
$$

$$
y_{k+1}
=
y_k + v_k\sin(\theta_k)\Delta t
$$

$$
\theta_{k+1}
=
\theta_k + \omega_k\Delta t
$$

Sehingga:

$$
\mathbf{x}_{k+1}
=
f(\mathbf{x}_k,\mathbf{u}_k)
$$

dengan:

$$
f(\mathbf{x}_k,\mathbf{u}_k)
=
\begin{bmatrix}
x_k + v_k\cos(\theta_k)\Delta t \\
y_k + v_k\sin(\theta_k)\Delta t \\
\theta_k + \omega_k\Delta t
\end{bmatrix}
$$

---

## 3. Contoh Perhitungan

Misalkan kondisi robot:

$$
x_k = 2.0\;m
$$

$$
y_k = 1.0\;m
$$

$$
\theta_k = 30^\circ
$$

dengan input:

$$
v_k = 0.5\;m/s
$$

$$
\omega_k = 0.2\;rad/s
$$

dan:

$$
\Delta t = 0.1\;s
$$

Maka:

$$
x_{k+1}
=
2.0 + 0.5\cos(30^\circ)(0.1)
$$

$$
x_{k+1}\approx2.0433\;m
$$

Untuk sumbu $Y$:

$$
y_{k+1}
=
1.0 + 0.5\sin(30^\circ)(0.1)
$$

$$
y_{k+1}=1.025\;m
$$

Orientasi:

$$
\theta_{k+1}
=
30^\circ + (0.2)(0.1)
$$

Karena $\omega$ menggunakan rad/s:

$$
\theta_{k+1}
\approx31.15^\circ
$$

Dengan demikian, prediksi keadaan robot menjadi:

$$
\mathbf{x}_{k+1}
\approx
\begin{bmatrix}
2.0433 \\
1.025 \\
31.15^\circ
\end{bmatrix}
$$

---

# 4. Jacobian Motion Model

Karena motion model bersifat nonlinear akibat fungsi $\sin(\theta)$ dan $\cos(\theta)$, EKF menggunakan Jacobian untuk melakukan linearisasi lokal.

Jacobian terhadap state adalah:

$$
\mathbf{F}
=
\frac{\partial f}{\partial\mathbf{x}}
$$

Untuk motion model di atas:

$$
\mathbf{F}
=
\begin{bmatrix}
1 & 0 & -v\sin(\theta)\Delta t \\
0 & 1 & v\cos(\theta)\Delta t \\
0 & 0 & 1
\end{bmatrix}
$$

Matrix $\mathbf{F}$ menunjukkan bagaimana perubahan kecil pada state sebelumnya memengaruhi state berikutnya.

Sebagai contoh, terdapat hubungan:

$$
\frac{\partial x_{k+1}}{\partial\theta_k}
=
-v_k\sin(\theta_k)\Delta t
$$

dan:

$$
\frac{\partial y_{k+1}}{\partial\theta_k}
=
v_k\cos(\theta_k)\Delta t
$$

Artinya, **ketidakpastian pada yaw $\theta$ akan memengaruhi prediksi posisi $x$ dan $y$**.

---

# 5. Process Noise

Motion model tidak pernah sempurna. Kesalahan dapat berasal dari:

* wheel slip
* perbedaan diameter roda
* wheelbase yang tidak akurat
* encoder error
* kesalahan kecepatan roda
* permukaan lantai
* perubahan beban robot
* kesalahan model kinematika

Oleh karena itu EKF menggunakan process noise covariance:

$$
\mathbf{Q}
$$

Prediksi covariance dilakukan dengan:

$$
\mathbf{P}_{k+1}^{-}
=
\mathbf{F}_k
\mathbf{P}_k
\mathbf{F}_k^T
+
\mathbf{Q}_k
$$

di mana:

* $\mathbf{P}$ = covariance state
* $\mathbf{F}$ = Jacobian motion model
* $\mathbf{Q}$ = process noise covariance

Semakin besar nilai $\mathbf{Q}$ pada suatu state, semakin besar ketidakpastian model terhadap prediksi state tersebut.

---

# 6. Hubungan dengan Sensor Fusion

Pada robot differential drive, aliran sensor fusion dapat digambarkan sebagai:

```text
Wheel Encoder
     │
     ▼
Wheel Kinematics
     │
     ├── v
     └── ω
     │
     ▼
Motion Model
     │
     ▼
EKF Prediction
     │
     ├── x
     ├── y
     └── θ
     │
     │
     ├─────────────── IMU
     │                  │
     │                  ▼
     │             Measurement
     │                Update
     │                  │
     └──────────────────┘
             │
             ▼
       Fused Odometry
```

Pada sistem ini, encoder digunakan untuk mendapatkan kecepatan linear dan angular robot:

$$
v = \frac{v_R+v_L}{2}
$$

$$
\omega = \frac{v_R-v_L}{L}
$$

dengan:

* $v_R$ = kecepatan roda kanan
* $v_L$ = kecepatan roda kiri
* $L$ = jarak antara roda kiri dan kanan

Kemudian $v$ dan $\omega$ digunakan oleh motion model untuk memprediksi:

$$
x,\;y,\;\theta
$$

IMU kemudian memberikan informasi tambahan, misalnya yaw dan angular velocity, yang digunakan pada tahap **measurement update** EKF.

---

# 7. Ringkasan

Secara sederhana, proses sensor fusion dapat ditulis:

$$
Encoder
\rightarrow
(v,\omega)
\rightarrow
Motion\ Model
\rightarrow
Prediction
$$

kemudian:

$$
Prediction
+
IMU
\rightarrow
EKF\ Measurement\ Update
$$

Sehingga EKF menghasilkan estimasi:

$$
\boxed{
\mathbf{x}
=
[x,\;y,\;\theta]^T
}
$$

yang merupakan estimasi posisi dan orientasi robot setelah menggabungkan informasi dari model gerak dan sensor.


**Reference**

* Hartzer, & Saripalli. (2025). *EKF_CAL: Extended Kalman Filter-based Calibration and Localization*. Journal of Open Source Software, 10(109), 7793.
  [Paper PDF](https://joss.theoj.org/papers/10.21105/joss.07793)