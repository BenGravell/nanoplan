# TODO

--
Put this info on the Tutorial page. Probably need separate pages for introduction and keymap/controls.

INTRODUCTION
The ego and traffic race on various circuits.

- **track** selects the seeded circuit, a built-in preset, or a downloaded circuit.
- **planner** changes the active motion planner.
- **future preview** sets how many seconds of the current plan are drawn;
  zero hides the preview without stopping the ego.
- **diagnostic points/trajectories** show the selected planner's sampled
  search geometry when that planner records diagnostics.
- **pause** freezes the simulation.
- **new track** increments the seed and spectrally regenerates a simple closed
  circuit, its width, traffic, and the ego start.
- **scroll** zooms the camera.

--
Taxonomy of planners
- Sampling based (MPPI)
- Tree search (RRT)
- Local optimization (iLQR)

--
Focus on getting planners performing well when no other actors are present.
We should see racelines (setting up outside of a corner and hitting the apex of the turn) emerge naturally. 

## Planning


Frenet lattice

Sample in space of s,t,v,time

s: progress coord
t transverse coord
v: speed
Time

Take yaw from frenet i.e. zero in frenet angle, so we are not sampling over yaw angles.

Coarse grid, discretize with about 5 to 8 breakpoints per dimension. Total traj segments 1000.


Sampling bounds

Use road width for T coord bound.

Use max throttle/brake to set S coord bounds.

Use accel limits for speed bounds.

Time sampled at every 1second (10 ticks)


Discard kinematic infeasible traj after performing frenet transform.

Then run metric objective and use that as edge cost in A star.

## Actor planning

--
Fix the actors. They should run a basic planner instead of using magic unphysical motion.

--
Left-menu tab with more options for the opponents:

1. Planner to use for opponents.
2. Personality characteristics.
  - Assertiveness - progress weight
  - Recklessness - safety weight

## guidance mode

Add guidance mode, human steers target for planner

## planning horizon

planning horizon might be too long, seems to cause bad behaviors like flickering and slowdown.

need to handle progress rewards somehow elegantly so that we encourage short-term acceleration without becoming too myopic and failing to reason about and anticipate corners and overtake maneuvers 

## Cost map

- Compute signed distance field to obstacles and road boundaries. Then take Euclidean distance transform to get a proximity cost map. This can be used for the collision and proximity costs and metrics.
This works for static obstacles.

## UX
--

road window seems to draw over itself on short tracks [Test Track (small)]
results in weird doubled up station lines

-- New flow for driving startup.

Start -> Track Select

Choose a track.
Horizontal gallery of tracks.
Each shown as a minimap thumbnail centered in square lockup with track name caption below.
Gallery is at bottom of screen, about bottom 20 percent.
Top 80 percent dedicated to track big map display preview and details / stats about the track:
Length
Number of turns/corners
Average/min/max curvature

After selecting track then dive into the driving app mode.

--
Show the predicted future poses of actors in the viewer.
Re-use the ego carpet element.
Must stay lightweight on compute and rendering side.
We don't need full coloration, can use simplified single color grey mesh.
Add checkbox in VIZ options for showing them.

--
Give nanoplan a unique icon/favicon.
Display the favicon on the website app ( browser tab), both local and cloud deployed.
Use a combination of AI generation and open source iconography according to Best design principles a la Allan Peters.

Symbology:
steering wheel with 45 degree chamfer corners, in a square lockup.

--
Add timescrubbers for freezing simulation and replaying past.

Make the timescrubbers bigger with big touch targets for the grab handles.
Use the full page width.
Put the at the bottom of the screen in dedicated area/container like a video player would have.

--
Give the various actors in the scene minimal meshes representative of the semantic class (car, truck, cycle, pedestrian, etc)
