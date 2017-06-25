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

6. 
