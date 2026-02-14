# 2D Differential-Drive Robot Simulation

A browser-based robotics sandbox that simulates a differential-drive robot in a 2D world.  
The robot uses simple local sensing and a reactive controller to move through obstacles and
seek the brightest area in the environment.

## What This Project Demonstrates

- Differential-drive motion with independent left/right wheel speeds
- Obstacle sensing using forward ray-style distance checks
- Light-seeking behavior using three directional brightness samples
- A blended controller that balances exploration, obstacle avoidance, and escape behavior
- Real-time visualization of world state, robot path, sensors, and key telemetry

## Simulation Overview

Each generated map contains:

- rectangular obstacles placed at random valid locations
- one light source ("brightest spot") with a Gaussian intensity profile
- one robot initialized in a valid non-colliding start pose

At each animation step:

1. The robot samples three forward sensor directions (left, center, right).
2. Each sensor reports:
   - nearest obstacle/wall distance along that direction
   - local brightness ahead of that direction
3. The controller computes linear and angular motion tendencies from:
   - brightness gradient (`right - left`) to steer toward brighter regions
   - center brightness to increase/decrease forward speed
   - obstacle proximity to reduce speed and steer away from collisions
4. Wheel speeds are clamped to physical limits, and robot state is integrated over `dt`.
5. The frame is rendered and telemetry is updated.

## Tech Stack

- Node.js HTTP server (no framework)
- Vanilla JavaScript for simulation and rendering
- HTML5 Canvas for visualization
- Plain CSS for UI styling

## Project Structure

- `server.js`: static file server and local entry point
- `index.html`: page layout and controls
- `app.js`: simulation world, sensing, control logic, and rendering loop
- `styles.css`: visual styling for the UI and canvas container

## Getting Started

### Prerequisites

- Node.js 18+ (or any modern Node runtime that supports this code)

### Install and Run

```bash
npm install
npm start
```

Open:

`http://localhost:3000`

If needed, you can change the port using:

```bash
PORT=4000 npm start
```

On PowerShell:

```powershell
$env:PORT=4000; npm start
```

## Controls

- `Pause / Resume`: stop or continue the simulation loop
- `Reset Robot`: respawn the robot on the current map
- `New Map`: regenerate obstacles and light position, then respawn robot

## Telemetry Display

The stats panel shows:

- elapsed simulation time
- robot position and heading
- wheel speeds (left/right)
- current and best brightness values reached
- distance to the light source

These values are useful when tuning controller gains or comparing behavior across maps.

## Notes on Behavior

- Obstacle avoidance is reactive and local, not global path planning.
- The robot can occasionally stall or oscillate in difficult geometries; escape logic helps it recover.
- Because maps and initial poses are randomized, each run looks different.

## Future Improvements (Ideas)

- add deterministic seeds for reproducible experiments
- expose controller gains via UI sliders
- add multiple robots and comparative controllers
- log metrics to CSV/JSON for offline analysis
- add goal-complete criteria and benchmark timings
