# Mathematical Derivations: 2nd-Order Thevenin ECM & Extended Kalman Filter

This document provides complete continuous-to-discrete mathematical proofs and derivations for the Battery Management System algorithms.

---

## 1. 2nd-Order Thevenin Equivalent Circuit Model (ECM)

A Lithium-ion cell is modeled using an electro-thermal 2nd-order Thevenin circuit consisting of:
1. **$V_{OC}(SoC, T)$**: Non-linear open-circuit voltage source.
2. **$R_0(T)$**: Ohmic internal resistance (electrolyte, separator, and current collector resistance).
3. **$R_1 \parallel C_1$**: Fast polarization branch (charge transfer resistance & double-layer electrochemical capacitance, $\tau_1 = R_1 C_1 \approx 12.75\text{ s}$).
4. **$R_2 \parallel C_2$**: Slow polarization branch (solid-state lithium diffusion in active materials, $\tau_2 = R_2 C_2 \approx 72.0\text{ s}$).

```
   (+) o────────[ R0 ]───────┬──────[ R1 ]──────┬──────┬──────[ R2 ]──────┬───────o Terminal (+)
                             │                  │      │                  │
                            === C1             [ ]    === C2             [ ]
                             │                  │      │                  │
                             └──────────────────┘      └──────────────────┘
   (-) o───────────( Voc(SoC) )───────────────────────────────────────────────────o Terminal (-)
```

### Continuous-Time Differential Equations

Let the continuous state vector be:
$$x(t) = \begin{bmatrix} SoC(t) \\ V_1(t) \\ V_2(t) \end{bmatrix}$$

Under load current $I_L(t)$ (defined as $I_L > 0$ for discharge, $I_L < 0$ for charge):
$$\frac{d}{dt} SoC(t) = -\frac{\eta}{Q_n} I_L(t)$$
$$\frac{d}{dt} V_1(t) = -\frac{1}{R_1 C_1} V_1(t) + \frac{1}{C_1} I_L(t)$$
$$\frac{d}{dt} V_2(t) = -\frac{1}{R_2 C_2} V_2(t) + \frac{1}{C_2} I_L(t)$$

where:
* $Q_n$ is the nominal cell capacity in Coulombs ($Q_n = C_{Ah} \times 3600$).
* $\eta$ is the coulombic efficiency ($\eta = 0.998$ during charging, $\eta = 1.0$ during discharging).

The continuous state-space representation is:
$$\dot{x}(t) = A_c x(t) + B_c I_L(t)$$

$$A_c = \begin{bmatrix} 0 & 0 & 0 \\ 0 & -\frac{1}{\tau_1} & 0 \\ 0 & 0 & -\frac{1}{\tau_2} \end{bmatrix}, \quad B_c = \begin{bmatrix} -\frac{\eta}{Q_n} \\ \frac{1}{C_1} \\ \frac{1}{C_2} \end{bmatrix}$$

The non-linear measurement equation for terminal voltage $V_t(t)$ is:
$$V_t(t) = V_{OC}(SoC(t)) - V_1(t) - V_2(t) - I_L(t) R_0(T)$$

---

## 2. Exact Discrete-Time State-Space Discretization (ZOH)

For digital microcontroller implementation with sample period $\Delta t$, we apply Zero-Order Hold (ZOH) exact integration assuming $I_L(t)$ is constant over the interval $[k\Delta t, (k+1)\Delta t)$:

$$A_d = e^{A_c \Delta t} = \begin{bmatrix} e^0 & 0 & 0 \\ 0 & e^{-\Delta t / \tau_1} & 0 \\ 0 & 0 & e^{-\Delta t / \tau_2} \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \alpha_1 & 0 \\ 0 & 0 & \alpha_2 \end{bmatrix}$$

where $\alpha_1 = \exp(-\Delta t / \tau_1)$ and $\alpha_2 = \exp(-\Delta t / \tau_2)$.

$$B_d = \int_{0}^{\Delta t} e^{A_c \tau} B_c \, d\tau = \begin{bmatrix} -\frac{\eta \Delta t}{Q_n} \\ \frac{1}{C_1} \int_{0}^{\Delta t} e^{-\tau / \tau_1} d\tau \\ \frac{1}{C_2} \int_{0}^{\Delta t} e^{-\tau / \tau_2} d\tau \end{bmatrix} = \begin{bmatrix} -\frac{\eta \Delta t}{Q_n} \\ R_1 (1 - \alpha_1) \\ R_2 (1 - \alpha_2) \end{bmatrix}$$

This yields the exact, stable discrete linear state equation:
$$x_{k+1} = A_d x_k + B_d I_{L,k} + w_k, \quad w_k \sim \mathcal{N}(0, Q)$$

---

## 3. Discrete Extended Kalman Filter (EKF) Formulation

Because $V_{OC}(SoC)$ is non-linear, the standard Kalman filter is extended by computing the first-order Taylor expansion (Jacobian) at each time step.

### Step 1: Time Update (State & Covariance Prediction)

$$
\hat{x}_{k|k-1} = A_d \hat{x}_{k-1|k-1} + B_d I_{L,k}
$$

$$
P_{k|k-1} = A_d P_{k-1|k-1} A_d^T + Q
$$

where $Q = \text{diag}([q_{soc}, q_{v1}, q_{v2}])$ is the process noise covariance matrix representing current integration drift and RC modeling approximations.

### Step 2: Measurement Prediction

$$
\hat{V}_{t,k} = V_{OC}(\hat{SoC}_{k|k-1}) - \hat{V}_{1,k|k-1} - \hat{V}_{2,k|k-1} - I_{L,k} R_0
$$

### Step 3: Measurement Linearization (Jacobian Matrix $C_k$)
The measurement matrix $C_k \in \mathbb{R}^{1 \times 3}$ is the partial derivative of terminal voltage with respect to states:

$$
C_k = \left. \frac{\partial V_t}{\partial x} \right|_{\hat{x}_{k|k-1}} = \begin{bmatrix} \left.\frac{\partial V_{OC}}{\partial SoC}\right|_{\hat{SoC}_{k|k-1}} & -1 & -1 \end{bmatrix}
$$

### Step 4: Innovation and Kalman Gain
The innovation scalar $y_k$ and innovation covariance $S_k$ are:

$$
y_k = V_{t,measured,k} - \hat{V}_{t,k}
$$

$$
S_k = C_k P_{k|k-1} C_k^T + R_{meas} \quad \in \mathbb{R}
$$

Because $S_k$ is a scalar ($1 \times 1$), matrix inversion is reduced to scalar division:

$$
K_k = \frac{P_{k|k-1} C_k^T}{S_k} \quad \in \mathbb{R}^{3 \times 1}
$$

### Step 5: State Correction

$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k y_k
$$

$$
\hat{SoC}_{k|k} = \text{clamp}(\hat{x}_{1,k|k}, 0.0, 1.0)
$$

### Step 6: Joseph Form Covariance Update (Numerical Stability Proof)
In finite-precision fixed-point or single-precision embedded arithmetic, the conventional update $P = (I - K C) P_{pred}$ can lose positive definiteness due to round-off subtraction errors, causing filter divergence.

We employ the **Joseph Form Covariance Update**:

$$
P_{k|k} = (I - K_k C_k) P_{k|k-1} (I - K_k C_k)^T + K_k R_{meas} K_k^T
$$

**Mathematical Proof of Guaranteed Positive Semi-Definiteness:**
1. Let $M = (I - K_k C_k)$.
2. For any non-zero vector $z \in \mathbb{R}^3$:

   $$
   z^T P_{k|k} z = z^T \left( M P_{pred} M^T + K_k R K_k^T \right) z = (M^T z)^T P_{pred} (M^T z) + R_{meas} (K_k^T z)^2
   $$

3. Since $P_{pred} \succ 0$ and $R_{meas} > 0$:

   $$
   (M^T z)^T P_{pred} (M^T z) \ge 0 \quad \text{and} \quad R_{meas} (K_k^T z)^2 \ge 0
   $$

4. Therefore, $z^T P_{k|k} z \ge 0$ unconditionally for all time steps $k$, preventing negative eigenvalues.

---

## 4. Recursive Least Squares (RLS) Online SoH Identification

To track battery aging online, the ohmic internal resistance $R_0$ is identified continuously.

From the ECM output equation:

$$
V_{OC}(SoC) - V_1 - V_2 - V_t = I_L \cdot R_0
$$

Let:
* Output measurement: $y_{k} = V_{OC}(\hat{SoC}_k) - \hat{V}_{1,k} - \hat{V}_{2,k} - V_{t,k}$
* Regressor: $\phi_k = I_{L,k}$
* Unknown parameter: $\theta = R_0$

Using an exponential forgetting factor $\lambda = 0.9995$ to prioritize recent aging dynamics:

$$
e_k = y_k - \phi_k \hat{\theta}_{k-1}
$$

$$
K_{RLS,k} = \frac{P_{k-1} \phi_k}{\lambda + \phi_k^2 P_{k-1}}, \quad \hat{\theta}_k = \hat{\theta}_{k-1} + K_{RLS,k} e_k
$$

$$
P_k = \frac{1}{\lambda} \left( P_{k-1} - K_{RLS,k} \phi_k P_{k-1} \right)
$$

### State of Health (SoH) Definition
Automotive standard IEC 62660 defines battery End of Life (EOL) when internal resistance doubles ($R_{0,EOL} = 2.0 \cdot R_{0,BOL}$):

$$
SoH_{R0} = \max\left(0, \min\left(100, \left(1 - \frac{\hat{R}_0 - R_{0,BOL}}{R_{0,BOL}}\right) \times 100\%\right)\right)
$$
