particle_filters
================

a little robot I built for an ai for robotics course

The robot is randomly placed into a world with several landmarks. It also initializes a random distribution of hypothedical robots into the same space.  To figure out where it is located it measures the distance to itself and the landmarks, then, with prejudice, samples the likely robots that would have the same measurement.  It then moves, and repeats the experiement to come up with an estimation of its location in the world. 


I originally did not create a visual for this program, but thought it may be helpful - the visual breaks a few OOP rules (circular reference) but I felt it necessary to add this piece in to demonstrate what the code is doing.

To run the code simply call draw_robot.py
