# Overview
This is a program that simulates the random movement of particles. The latters are represented by non-overlapping circles and this condition is ensured with the use of a quadtree.

# Requirement
- Python 3.8

# Demos
## Trajectory of a particle
<div>
  <img src="Demo/particle_demo1.png" align="center">
  <figcaption>**Figure 1. Circle zone with radius equal to 250 pixels containing 200 small circles with radius equal to 10 pixels.** The circle with the green outline is the starting position and the green circle is the final position. The blue rectangle represents the the hitbox of the movement of the green circle and the red circles are the obstacles. The squares are the partions of a quadtree.
  </figcaption>
</div>

## Moving particles
<div>
  <img src="Demo/particle_demo2.gif" align="center">
  <figcaption> **Figure 2. Circle zone with radius equal to 250 pixels containing 50 small circles with radius qual to 10 pixels.** The positions of the circles are configured to be updated at the rate of 60 frames per second (this might not be accurate because the "after" function of tkinter is used to animate the movement).
  </figcaption>
</div>

