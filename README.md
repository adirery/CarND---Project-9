# Model Predictive Control

## Overview
The MPC algorithm works on the basis of the data stream provided by the udacity simulator. This is a JSON object consisting of:
- X-Points: X-coordinates of the waypoints the car should follow in "global" coordinates
- Y-Points: Y-coordinates of the waypoints the car should follow in "global" coordinates
- X-Position: The car's X-Position
- Y-Position: The car's Y-Position
- Psi (Angle): The car's angle
- Velocity: The car's velocity

The processing steps are the following:
1. Transform X/Y Points from "global" to "local" coordinates (i.e. to the car's coordinate system). This is done using the following transformation (transformGlobalToLocal function in main.cpp):
```
 X (local) =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
 Y (local) =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);  
```

2. A 3rd degree polynomial is fitted through the transformed waypoints (using polyfit in main.cpp)

3. The fitted curve is evaluated against the waypoints, resulting in the cross-track-error (cte) (using polyeval in main.cpp)

4. The fitted curve direction (first derivate of the  is evaluated against the car heading, resulting in psi-error (epsi). Since the heading of the car in car's coordinates is always 0, i.e. psi = 0, the function is epsi = psi - atan(coeffs[1]) --> epsi = 0 - atan(coeffs[1])

5. The state of the car is created as a vector [x, y, psi, v, cte, epsi]. Since x, y and psi are 0 in the car's coordinates the vector simplifies to [0, 0, 0, v, cte, epsi]

6. The MPC algorithm is run (further details below) to calculate the optimal trajectory and related steering & acceleration to follow the fitted curve calculated above.

7. The steering value is transformed from degrees (result of the MPC calculation) to rad (needed for the simulator actuator)

8. The waypoints and the MPC results are pushed into the respective vectors (next_..._vals and mpc_..._vals resectively)

## MPC Algorithm
The MPC algorithm implemented follows pretty much the algorithm outlined in the lectures with one addition: The previous steering & acceleration values are provided from the main.cpp in order to cater for the 100ms delay between calculation and actuation.

In particular this means the following main steps:
1. 

### Cost function

The cost function to be minimized consists of the following:
```
+ Sum of the cross-track-error for all the steps
+ Sum of the heading error for all the steps
+ Sum of the speed error towards the reference speed for all the steps
+ Sum of the actuators (steering & acceleration)
+ Sum of the actuator changes between two consequent time steps (steering & acceleration)
```

### Constraints
The car is constrained by its physical model:
```

```

### Hyperparameters & Tuning
- N: Number of steps the MPC algorithm "predicts" into the future. Initially I had this value at 10, but the results were very bad. Pushing it up to 20 resulted in much cleaner / smoother driving behaviour, especially in curves.
- dt: Time step between each predicted step. In order to drive smoothely this value was set to 0.05.

### Additional tuning
In addition to the Hyperparameter tuning, the cost function, specifically the steering "smoothing" part was given much more emphasis by adding a factor of 600.
