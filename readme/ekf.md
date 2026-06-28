# Extended Kalman Filter (EKF)

## The Big Picture

Imagine you are driving blindfolded and someone is giving you two pieces of information:

1. A **physicist friend** who knows your speed and direction and says *"based on your last position and how fast you've been going, you should be HERE now"*.
2. A **GPS device** that occasionally pings and says *"I see you HERE"* (but it's noisy and sometimes off by a few meters).

You wouldn't blindly trust either one. You'd weigh them — trust the physicist when the GPS is clearly wrong, trust the GPS when you've been driving for too long without a fix. **That weighing process is exactly what the EKF does.**

---

## The State Vector

Before anything, we need to define *what* we are estimating. In this project, the state is:

$$\mathbf{x} = [p_x,\ p_y,\ p_z,\ v_x,\ v_y,\ v_z,\ \phi,\ \theta,\ \psi]^T \quad (9 \times 1)$$

| Symbol | Meaning |
|--------|---------|
| $p_x, p_y, p_z$ | 3D position of the vehicle |
| $v_x, v_y, v_z$ | 3D velocity of the vehicle |
| $\phi, \theta, \psi$ | Roll, pitch, yaw (orientation) |

Think of this as the robot's "belief" about itself at any point in time — a single vector that summarizes everything we know.

---

## The Two Steps of EKF

The EKF alternates between two steps, over and over, at every timestep:

```
+--------------+       +------------------+
|   PREDICT    |------>|     UPDATE       |
|  (Physics)   |       |  (Sensor Fusion) |
+--------------+       +------------------+
       ^                        |
       +------------------------+
              (repeat every dt)
```

---

## Step 1 — Predict: "Where should I be based on physics?"

The predict step answers: *given where I was and how fast I was going, where am I now?*

### The Motion Model

We use a simple constant-velocity kinematic model, encoded as a **state transition matrix** F:

$$\mathbf{x}_{k|k-1} = F \cdot \mathbf{x}_{k-1|k-1}$$

The key entries in F are the off-diagonal `dt` terms:

- `F(0,3) = dt` encodes: `px_new = px_old + vx * dt`
- `F(1,4) = dt` encodes: `py_new = py_old + vy * dt`
- `F(2,5) = dt` encodes: `pz_new = pz_old + vz * dt`

This is just **kinematics** — position updates using velocity. Velocities and orientations are assumed constant between timesteps (they'll be corrected in the update step).

### Predicting the Covariance

$$P_{k|k-1} = F \cdot P_{k-1|k-1} \cdot F^T + Q$$

| Term | Intuition |
|------|-----------|
| $P$ | **"How unsure am I?"** — a 9x9 matrix capturing our uncertainty about every element of the state, and how errors in one dimension correlate with others |
| $F \cdot P \cdot F^T$ | Propagates existing uncertainty through the motion model — if I was uncertain about velocity, now I'm also uncertain about the new position |
| $Q$ | **Process noise** — uncertainty we add every step because our model is imperfect (the real world isn't perfectly constant-velocity) |

> **Intuition for P (Covariance):** Think of P as the **numerical memory of all past errors**. If the filter has been drifting to the right for the past 5 seconds, P gets large in the `px` direction — it "remembers" it can't trust itself in that direction. A large P value means *"I've been wrong here before, take my prediction with a grain of salt."*

---

## Step 2 — Update: "What does the sensor say, and how much do I trust it?"

Now a sensor arrives (ICP, GPS, or IMU). This is where we reconcile our physics-based prediction with the real world.

### The Measurement Matrix H

A sensor doesn't observe the full state. It only sees a slice of it. The **measurement matrix** H is a linear map from the full state space to what the sensor actually measures:

**GPS** measures only position → H_GPS is 3x9:
```
     px py pz vx vy vz roll pitch yaw
H = [ 1  0  0  0  0  0   0    0    0 ]
    [ 0  1  0  0  0  0   0    0    0 ]
    [ 0  0  1  0  0  0   0    0    0 ]
```

**IMU** measures only orientation → H_IMU is 3x9:
```
     px py pz vx vy vz roll pitch yaw
H = [ 0  0  0  0  0  0   1    0    0 ]
    [ 0  0  0  0  0  0   0    1    0 ]
    [ 0  0  0  0  0  0   0    0    1 ]
```

**ICP** (LiDAR) measures position + orientation → H_ICP is 6x9.

### Innovation — "The Surprise"

$$\mathbf{y} = \mathbf{z} - H \cdot \mathbf{x}_{k|k-1}$$

| Term | Intuition |
|------|-----------|
| $\mathbf{z}$ | What the sensor actually measured |
| $H \cdot \mathbf{x}$ | What we *expected* the sensor to measure, based on our prediction |
| $\mathbf{y}$ | The **innovation** — the *surprise*. How far off was our prediction from reality? |

- If `y ≈ 0`, the sensor agrees with our prediction — nothing much to update, we were right.
- If `y` is large, there's a discrepancy — we need to correct our state.

The innovation is the engine of the filter. Without it, we'd just be integrating kinematics (dead reckoning) and drifting forever.

### Innovation Covariance S

$$S = H \cdot P \cdot H^T + R$$

| Term | Intuition |
|------|-----------|
| $H \cdot P \cdot H^T$ | How uncertain our *predicted measurement* is (our model's uncertainty, projected into sensor space) |
| $R$ | **Measurement noise** — how noisy the sensor itself is (a fixed property of the sensor hardware) |
| $S$ | Total uncertainty in the innovation: *"How surprising should a large innovation actually be?"* |

> **R vs Q:** Q is how much we distrust our motion model. R is how much we distrust the sensor. Both are diagonal matrices here — each diagonal entry is the variance (squared expected error) of one dimension.

### The Kalman Gain K — The Weighting Factor / Referee

$$K = P \cdot H^T \cdot S^{-1}$$

This is the most important equation. K answers: *"What fraction of the innovation should I actually believe?"*

- If **P is large** (we are very uncertain about our state) → K is large → **trust the sensor more**.
- If **R is large** (the sensor is very noisy) → S is large → K is small → **trust the prediction more**.
- If **P is small** (we are very confident in our prediction) → K is small → **trust the sensor less**.

Think of K as a **confidence competition** between your physics model and your sensor. The Kalman gain is the referee.

### State Update

$$\mathbf{x}_{k|k} = \mathbf{x}_{k|k-1} + K \cdot \mathbf{y}$$

We nudge our predicted state by `K * y`:
- A large, trusted innovation (K large, y large) → big correction.
- A small, distrusted innovation (K small) → barely move.

### Covariance Update

$$P_{k|k} = (I - K \cdot H) \cdot P_{k|k-1}$$

After incorporating a sensor measurement, we become **more confident** — P shrinks. The more trustworthy the sensor (low R), the more P shrinks. This makes sense: receiving a reliable measurement reduces your uncertainty.

> **Full cycle intuition for P:** Each predict step *inflates* P (physics propagates uncertainty and adds Q). Each update step *deflates* P (new information reduces uncertainty). The filter breathes: **expand → contract → expand → contract.**

---

## Why "Extended"? Jacobians and Linearization

A standard Kalman Filter only works for **linear** systems. But the real world is not linear — especially for rotation.

Consider expressing 3D orientation as a rotation matrix `R(roll, pitch, yaw)`. This is a nonlinear function of the angles. If you want to propagate uncertainty through it (for P), you can't just multiply matrices — the math breaks.

### The Fix: Linearize with the Jacobian

The Jacobian is the multidimensional generalization of a derivative. For a nonlinear function `f(x)`, the Jacobian J_f is the matrix of all partial derivatives:

$$J_f = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix}
\frac{\partial f_1}{\partial x_1} & \cdots & \frac{\partial f_1}{\partial x_n} \\
\vdots & \ddots & \vdots \\
\frac{\partial f_m}{\partial x_1} & \cdots & \frac{\partial f_m}{\partial x_n}
\end{bmatrix}$$

**Intuition:** A derivative tells you *"if I move a tiny bit in this direction, how does the output change?"* The Jacobian does this for every input and every output simultaneously. It is a **local linear approximation** of the nonlinear function — valid near the current operating point.

### How It Linearizes the System (Taylor Expansion)

For a nonlinear system `x_k = f(x_{k-1})`, the EKF approximates using a first-order Taylor expansion:

$$f(\mathbf{x}) \approx f(\mathbf{x}_0) + J_f \Big|_{\mathbf{x}_0} \cdot (\mathbf{x} - \mathbf{x}_0)$$

We replace the nonlinear `f` with a **tangent plane** at the current best estimate `x_0`. This is accurate as long as uncertainty is small (i.e., we don't stray far from `x_0`).

```
 f(x)   nonlinear function
   |           o~~~o
   |       o~~~      ~~~o
   |   o~~~
   | /  <--- Jacobian (tangent line at x0)
   |/
   +----+--+--+--+------ x
            x0
```

The Jacobian J_f then replaces F in the covariance prediction equation:

$$P_{k|k-1} = J_f \cdot P_{k-1|k-1} \cdot J_f^T + Q$$

Similarly, if the measurement function `h(x)` is nonlinear (e.g., bearing-angle cameras), we compute J_h and use it in place of H.

> **In this project:** The motion model is already linear (constant velocity), so F is exact — no Jacobian needed for prediction. The sensors (GPS, IMU, ICP) provide measurements that already live in state space, so H is also linear and exact. In a more complex EKF (e.g., fusing camera bearing measurements or full IMU pre-integration), you would compute Jacobians at every single timestep.

---

## Complete EKF Cycle — Summary

```
                   +--------------------------------------+
INITIALIZATION     |  x0 = [first ICP pose]               |
                   |  P0 = 0.1 * I (low initial uncertainty)|
                   +-------------------+------------------+
                                       |
                   +-------------------v------------------+
    PREDICT        |  x_pred = F * x         (kinematics) |
    (every dt)     |  P_pred = F*P*Ft + Q  (uncertainty up)|
                   +-------------------+------------------+
                                       |
                   +-------------------v------------------+
    UPDATE         |  y = z - H*x_pred      (innovation)  |
    (on sensor)    |  S = H*P*Ht + R    (total variance)  |
                   |  K = P*Ht*S^-1       (Kalman gain)   |
                   |  x = x_pred + K*y  (state update)    |
                   |  P = (I - K*H)*P  (uncertainty down) |
                   +-------------------+------------------+
                                       |
                                  (loop forever)
```

---

## Sensor Fusion in This Project

Three sensor modalities are fused:

| Sensor | Measures | Noise Param | Strength | Weakness |
|--------|----------|-------------|----------|----------|
| **ICP (LiDAR)** | Position + Orientation (6D) | `icp_noise` | Very accurate locally | Slow, accumulates drift globally |
| **GPS** | Position only (3D) | `gps_noise` | Global anchor, prevents drift | Noisy (+-3m synthetic noise added) |
| **IMU** | Orientation only (3D) | `imu_noise` | Fast, direct angle measurement | Gyro bias causes rotational drift |

The EKF naturally handles sensors of different dimensionality — each sensor has its own H matrix that "picks out" the relevant rows of the state vector.

---

## Parameter Tuning Guide

There are three knobs you control. Everything else in the EKF is deterministic math.

| Parameter | Config Key | Meaning |
|-----------|-----------|---------|
| **Process noise** | `EKF_PROCESS_NOISE` | How much you distrust your own motion model per timestep (added to Q diagonal) |
| **Sensor noise** | `EKF_ICP_NOISE` / `EKF_GPS_NOISE` / `EKF_IMU_NOISE` | How much you distrust each sensor (sets R diagonal) |
| **Initial P** | Hardcoded `0.1 * I` | How confident you are in the very first state estimate |

Getting them right is the art of Kalman filtering. There is no universal correct answer — it depends on your sensor hardware, environment, and speed of motion.

---

### The Mental Model Before You Touch Anything

Before adjusting numbers, understand the core trade-off:

```
High process_noise (Q)  →  P inflates fast between sensor updates
                        →  Kalman gain K grows
                        →  Filter aggressively trusts sensors
                        →  Noisy/jittery trajectory

Low process_noise (Q)   →  P barely inflates between sensor updates
                        →  Kalman gain K stays small
                        →  Filter barely reacts to sensors
                        →  Trajectory lags behind or ignores corrections
```

```
High sensor_noise (R)   →  S grows → K shrinks
                        →  Sensor is mostly ignored, prediction dominates
                        →  Safe from jitter, but slow to correct drift

Low sensor_noise (R)    →  S stays small → K grows
                        →  Sensor dominates every update
                        →  Fast corrections, but amplifies sensor outliers
```

The goal is to set Q and R to **match reality** — if your GPS has ±3 m noise, `EKF_GPS_NOISE` should reflect `3^2 = 9` (variance = std_dev²).

---

### Symptom → Cause → Fix

This is the fastest way to diagnose a poorly tuned filter by just looking at the trajectory:

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Trajectory is jittery / wiggles frame-to-frame | R too low (trusting noisy sensors too much) | Increase `sensor_noise` |
| Trajectory drifts and never corrects to GPS/ICP | R too high (ignoring sensors) or Q too low | Decrease `sensor_noise`, or increase `process_noise` |
| Filter reacts violently to a single bad reading | R too low, sensor outlier not rejected | Increase `sensor_noise`, add outlier gating |
| Trajectory looks smooth but drifts globally | Q too low — filter overconfident, not pulling toward GPS | Increase `process_noise` |
| Rotational drift only (position OK) | `EKF_IMU_NOISE` too high — IMU orientation corrections ignored | Decrease `EKF_IMU_NOISE` |
| Large jump at the very start | Initial P too high — filter overcorrects on first sensor update | Lower initial P in `init()` |

---

### Manual Starting Points

If you are tuning by hand, use the **variance = std_dev²** rule to initialize your R values, then adjust Q relative to them:

**Step 1: Estimate your sensor standard deviations**

- GPS accuracy: if you expect ±3 m position error → `EKF_GPS_NOISE = 9.0`
- ICP accuracy: if ICP gives ±0.1 m position error → `EKF_ICP_NOISE = 0.01`
- IMU accuracy: if IMU gives ±1° orientation error → `EKF_IMU_NOISE = ~0.0003` (in radians²)

**Step 2: Set process noise relative to your best sensor**

A good heuristic: `process_noise ≈ 1/10th of your best sensor noise`. If your best sensor is ICP at `0.01`, start with `process_noise = 0.001`.

**Step 3: Watch the innovation**

Run the filter and mentally track `y = z - H·x`. If innovations are consistently large, the filter is always surprised → your Q is too low or R too high. If innovations are always near zero but the trajectory still drifts, your R is too high and sensor corrections are being suppressed.

---

### Automated Tuning with Optuna

Manual tuning is tedious. This project includes [optuna_tuner.py](file:///d:/DeepLearning/Lidar-Localization/python/optuna_tuner.py) which treats parameter selection as a **black-box optimization** problem.

#### How it works

Each Optuna trial:
1. **Proposes** a set of noise parameters sampled from a log-uniform search space.
2. **Runs** the full C++ localization pipeline as a subprocess with those parameters.
3. **Evaluates** the result using the KITTI Relative Pose Error (RPE) metric: `objective = t_err% + λ * r_err`.
4. **Reports** the objective back to Optuna, which uses a Bayesian (TPE) sampler to propose smarter values next trial.

#### Search ranges (from the tuner source)

| Parameter | Search Range | Scale |
|-----------|-------------|-------|
| `EKF_PROCESS_NOISE` | `[1e-4, 1.0]` | log-uniform |
| `EKF_GPS_NOISE` | `[1e-3, 5.0]` | log-uniform |
| `EKF_ICP_NOISE` | `[1e-3, 1.0]` | log-uniform |

Log-uniform means the sampler explores `0.001, 0.01, 0.1, 1.0` with equal probability density — appropriate because noise values can span many orders of magnitude.

#### Running a tuning sweep

```bash
# Quick sweep: 50 trials, 200 frames per trial
./run_optuna.sh

# Custom sweep: more trials, more frames, tune rotation weight
python python/optuna_tuner.py \
    --n-trials 100 \
    --frames 500 \
    --rotation-weight 2.0
```

`--rotation-weight` (λ) controls whether you care more about translation or rotation accuracy. Setting λ > 1 penalises rotational drift more — useful for the EKF_IMU configuration which has known yaw drift.

#### Reading the Optuna output

After a sweep, three artefacts are saved:

| File | Contents |
|------|----------|
| `config/config_best.json` | Best parameter set, ready to use |
| `optuna_results.csv` | All trial results for offline analysis |
| `optuna_plots/param_importances.html` | Which parameters matter most |

The **parameter importance plot** is especially useful — it tells you which of the three noise values has the biggest impact on your metric. If `EKF_GPS_NOISE` importance is near 0, it means the filter barely uses GPS — a sign R is too high and GPS is being ignored regardless of its value.

---

## Worked Example: GPS Update

Assume current predicted state: `x = [10, 20, 0, 1, 0, 0, 0, 0, 0.5]^T`

GPS measurement arrives: `z_gps = [11, 21, 0.1]^T`

1. **Innovation:** `y = [11, 21, 0.1] - [10, 20, 0] = [1, 1, 0.1]`
   → GPS says we're ~1 m off in X and Y.

2. **If P is large** (covariance inflated from many predict steps without a sensor update):
   → K will be large → state moves most of the way toward the GPS reading.

3. **If R is large** (noisy GPS):
   → S inflates → K shrinks → state moves only a small fraction toward the GPS reading.

4. **After update:** P shrinks → we now know more. The next GPS update will have a smaller effect unless we drift again.

---

## References

- R. E. Kalman, *"A New Approach to Linear Filtering and Prediction Problems"*, 1960.
- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005. (Chapter 3 for EKF)
- J. Sola, *"A micro Lie theory for state estimation in robotics"*, 2018.
