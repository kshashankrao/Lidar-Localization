# How is GT created

Fusion of GNSS and IMU. The accuracy of GNSS is boosted by using base stations, it is now in cm accuracy.

# Representing the pose

It is represented by a 4 x 4 matrix (similar to a calib matrix).

## Why 4x4 matrix ?

The affine transformation is in the form Rx + T. While doing repeated multiplication of the rotation, it is efficient to represent this equation as a matrix. So this linear equation is represented as a homogenous matrix.

$$\begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \\ 0 & 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x \\ y \\ z \\ 1 \end{bmatrix} = \begin{bmatrix} r_{11}x + r_{12}y + r_{13}z + t_x(1) \\ r_{21}x + r_{22}y + r_{23}z + t_y(1) \\ r_{31}x + r_{32}y + r_{33}z + t_z(1) \\ 0x + 0y + 0z + 1(1) \end{bmatrix}$$

We need 4x4 matrix because:
1. To accomodate the addition of T to R. Notice that the last row allows the addition of T to the rotated point in the first 3 rows of the output.

2. It is better to have a square matrix for repeated multiplication (matrix chaining), to preserve the shape of the output.

## Evaluating Pose Estimation 

Unlike a simple start-to-finish distance check, the KITTI Odometry benchmark evaluates how consistently an algorithm performs over time. If an algorithm drifts to the left for 1 kilometer, and then drifts back to the right for 1 kilometer, it might have a perfect final position, but its overall trajectory is completely wrong.To prevent this "lucky drift" problem, KITTI uses a Sliding Window Relative Pose Error (RPE).

### The Sliding Window Method

KITTI breaks the entire driven trajectory into overlapping sub-segments of fixed lengths: 100m, 200m, 300m, 400m, 500m, 600m, 700m, and 800m.For every single valid segment in the dataset (from frame $i$ to frame $j$), it calculates the error between the Ground Truth (GT) and the Estimated pose. The final score is the average of all these sub-segments.

### Translation Error 

($t_{err}$): Measured as a percentage (%).Example: A 1.5% translation error means for every 100 meters driven, the estimated position drifts by 1.5 meters.

We extract the translation vector $\Delta \mathbf{t}$ (the top-right $3 \times 1$ column of $\Delta T$) and calculate its Euclidean distance (L2 norm):$$t_{err} = \| \Delta \mathbf{t} \|_2 = \sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2}$$ 

To find the error for a segment from frame $i$ to frame $j$, we first calculate the relative motion taken by both the GT and the Estimate, and then find the difference between them.

Let $T_{GT}$ be the $4 \times 4$ ground truth matrix, and $T_{Est}$ be the $4 \times 4$ estimated matrix. The error matrix $\Delta T$ is:$$\Delta T = (T_{GT, i}^{-1} \cdot T_{GT, j})^{-1} \cdot (T_{Est, i}^{-1} \cdot T_{Est, j})$$This $\Delta T$ is itself a $4 \times 4$ matrix. We extract the translation and rotation errors directly from it.


### Rotation Error ($r_{err}$): 

Measured in degrees per 100 meters (deg/100m).Example: A 0.02 deg/100m error means the heading (yaw) drifts by a fraction of a degree every block driven.

We extract the rotation matrix $\Delta R$ (the top-left $3 \times 3$ block of $\Delta T$). 

The angular error is derived from the trace (the sum of the main diagonal) of this matrix:$$r_{err} = \arccos\left( \frac{\text{Trace}(\Delta R) - 1}{2} \right)$$To get the final reported KITTI numbers, $t_{err}$ is divided by the segment length to get the percentage, and $r_{err}$ is divided by the segment length and multiplied by 100 to get the deg/100m standard.












