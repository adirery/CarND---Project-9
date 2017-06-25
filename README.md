
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
1. Transform X/Y Points from "global" to "local" coordinates (i.e. to the car's coordinate system). This is done using the following transformation:
```
 X (local) =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
 Y (local) =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);  
```
