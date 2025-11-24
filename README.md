# Ackermann-Steering-Geometry


https://github.com/user-attachments/assets/50cf1c7b-6a33-4283-9ffc-dc9372ad9f8d

---

Project simulates an **Ackermann Steering Geometry** using the PyBullet physics engine. The simulation includes real-time steering control, vehicle kinematics, ICC visualization, and dynamic wheel orientation.

---

## ⭐ Features

<img width="537" height="466" alt="Screenshot 2025-11-25 025701" src="https://github.com/user-attachments/assets/952672fa-a8fa-4262-9177-b9f81a874f6e" /> <img width="243" height="182" alt="image" src="https://github.com/user-attachments/assets/5e270b66-69ed-46a3-9fe8-563a030e83e6" />

### Ackermann Kinematics
- **Forward Ackermann** – computes angular velocity from steering input  
- **Inverse Ackermann** – computes left/right wheel steering angles  
- **ICC (Instantaneous Center of Curvature)** computation  

### Visual Rendering
- Vehicle body rendered 
- Wheels rendered 
- ICC line rendered 
- Turning radius circle rendered 

### Real-Time Interaction
Adjust simulation parameters using PyBullet sliders:
- **Vehicle velocity (v)**
- **Steering angle (degrees)**

### Optimized Drawing
Turning circles are redrawn only when the steering value changes.

---
