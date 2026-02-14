# 2D Differential-Drive Robot Simulation

This project simulates a 2D differential-drive robot that:

- navigates around rectangular obstacles,
- senses local brightness,
- steers toward the brightest location on the map.

## Run

```bash
npm start
```

Then open:

`http://localhost:3000`

## Controls

- `Pause/Resume`: stop or continue the simulation loop
- `Reset Robot`: place the robot at a new valid start location
- `New Map`: regenerate obstacles and brightest spot

## Behavior Model

- Robot kinematics use left/right wheel speeds (`vl`, `vr`) and wheelbase.
- Three forward-facing sensors (left, center, right) estimate:
  - obstacle distance along a ray,
  - local light intensity ahead of each sensor direction.
- Controller combines:
  - brightness gradient following (`right - left`),
  - center-brightness forward drive,
  - obstacle repulsion and emergency escape turns.
