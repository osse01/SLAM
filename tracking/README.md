# Tracking

---

## Extended Kalman Filter (EKF)
The Extended Kalman Filter (EKF) works similarly to the ordinary Kalman filter but linearizes the system around the current state to estimate the next. In this implementation, finite difference methods are used to approximate the Jacobians. While analytic derivatives could be computed, the central difference approach has proven effective in practice.

Below: The estimated positions of an RC-Car using EKF, alongside the microphones and the true trajectory.
![EKF results showing estimated RC-Car positions, microphones, and true trajectory](images/EKF.png)

---

## Particle Filter (PF)
The Particle Filter (PF) populates the area with particles and iteratively resamples them based on their likelihood, given the observed data. Over time, only the most probable particles remain.

A good estimate of the RC-Car position can be obtained by taking the weighted linear combination of the particle positions.

Below: The estimated positions of an RC-Car using PF, alongside the microphones and the true trajectory.
![PF results showing estimated RC-Car positions, microphones, and true trajectory](images/PF.png)

---

## Testing

### Motion Model
A Constant Velocity (CV) model is used, as it is straightforward to implement and suitable for slow-moving vehicles like the RC-Car. This model effectively captures the overall movement.

### Sensor Model
The sensor model is based on the collected sound data. Since each beep's arrival time was recorded by multiple microphones, a Time Difference Of Arrival (TDOA) model is used.

### Data Collection and Modeling
Microphones were placed in a room, and an RC-Car drove around, emitting beeps at approximately 2 Hz. The sound data was collected from the microphones and processed to determine when each microphone received each beep. Because the exact transmission time was unknown, pairwise differences between microphone times were calculated to produce delta times, which were then used in the TDOA model to estimate the RC-Car's position.

<img width="500" height="auto" alt="Room setup with microphones and RC-Car for TDOA data collection" src="https://github.com/user-attachments/assets/022c6ae2-1c0b-4fbc-a6b0-2efc7e6f9c5e" />
