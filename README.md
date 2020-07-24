# Overview
This is a program that simulates the random movement of particles. The latters are represented by non-overlapping circles and this condition is ensured with the use of a quadtree.

# Requirements
- Python 3.8

# Demos
## Trajectory of a particle
<div>
  <img src="Demo/particle_trajectory.png" align="center">
  <figcaption>
    <p align="justify">
      <b>Figure 1. Circular zone with radius equal to 250 pixels containing 200 small circles with radius equal to 10 pixels.</b> The circle with the green outline is the starting position and the green is the final position. The blue rectangle represents the hitbox of the movement of the green circle and the red circles are the obstacles (the moving particle will stop at the first obstacle). The squares are the partitions of a quadtree. 
    </p>
  </figcaption>
</div>

## Moving particles
<div>
  <img src="Demo/moving_particles.gif" align="center">
  <figcaption>
    <p align="justify">
      <b>Figure 2. Circular zone with radius equal to 250 pixels containing 50 small circles with radius equal to 10 pixels.</b> The positions of the circles are configured to be
      updated at the rate of 60 frames per second (this might not be accurate because the "after" function of tkinter is used to animate the movement).
    </p>
  </figcaption>
</div>

## Particle vision
<div>
  <img src="Demo/particle_vision.png" align="center">
  <figcaption>
    <p align="justify">
      <b>Figure 3. Circular zone with radius equal to 250 pixels containing 100 small circles with radius equal to 10 pixels.</b> The circle highlighted in blue has a field of
      view extending up to 200 pixels and 90 degree angle. The circles and quadrants highlighted in red are inside the field of view.
    </p>
  </figcaption>
</div>

