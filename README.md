# Missile-sim

A simple simulator that demonstrates basic mathematical principles in a real world scenario.

## What is Missile Sim.?

Missile Sim is a small project, employing a Kalman filter to simulate a missile effectively tracking a target.

## Additional Features

Along with simulating a simple missile/projectile, Missile Sim also simulates the following:

- Dynamic targets with partially linear movment
- Target state estimation
- Missile gimble FOV limits
- Global random sensor error for distance calculations
- Easy to edit burntime for missile motor
- Dynamic drag calculations applied based on missile cross-section and air density
- Randomized 3D vector for wind simulation, applied on all objects in the simulation
- Initial launch velocity
- 3D graph displaying missile possitions and intercepts (*the math took me way too long*)

## How to Use

Install the requirements through `pip` and run `track.py`

## TODO

- Add image based 3D projections for possible real world drone based target tracking system
- Track contrails
- Add better UI
