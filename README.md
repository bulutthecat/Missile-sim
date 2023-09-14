# Missile-sim
A simple simulator that demonstrates basic mathematical principles in a real world scenario.
## What is Missile Sim?
It is a small project I wrote, that applies a Kalman filter to simulate a missile leading a target.
## What other functions does Missile Sim posess?
Along with simulating a simple missile/projectile, Missile Sim also simulates the following:
- Dynamic Targets with partially linear movment
- Target state estimation
- Missile gimble FOV limits
- global random sensor error for distance calculations
- easy to edit burntime for Missile motor
- Dynamic drag calculations applied based on missile cross-section and air density
- randomized 3D vector for wind simulation, applied on all objects in the simulation
- Initial launch velocity
- 3D graph displaying missile possitions and intercepts
*the math took me way too long*
## how to use
install the requirments through pip and run track.py

## TODO
- Add image based 3D projections for possible real world drone based target tracking system
- track contrails
- add better UI
