# TODO


--
Web app load time blew up after we went to optimized builds, unusably slow (waited on mobile phone for several minutes, appeared hung)
Need new strategy to manage download payload size and startup time.

--
Add a minimap display to upper right corner of the track canvas area.

Use a simplified rep.
- Track width constant, simple polyline using track centerline. Can be decimated since minimap always render small, only need coarse samples every 5m.
- Ego and opponents each rendered as circular dots. Ego should always render drawn over top of opponents. Ego should be orange, opponents blue.
- Minimap should not rotate, use a constant position and rotation. Nothing fancy.


--
Put this info on the Tutorial page.
Probably need separate pages for introduction and keymap/controls.

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
expand the tutorial with more pictograms and a steady easy onboarding info flow

--
Taxonomy of planners
- Sampling based (MPPI)
- Tree search (RRT)
- Local optimization (iLQR)


--
Measure and reduce time to first display/user interaction on app load in web app / mobile.

Loaders (tracks, otherwise) should go in a background process that is non-blocking.
Start app with download-free procedural track.


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

-- Grid display

Minor headlines are invisible, do not show up.

Zoom based wide grid should be a perfect power 2 multiple of the normal zoom grid so that the lines don't pop position on zoom. Effect should be purely that lines become thicker/thinner and disappear/appear, not popping or shifting position.


--
Make chevron on start menu like 50% bigger.


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
Give ego and all actors in the scene minimal meshes representative of a racecar. pure cosmetic, keep rectangular collision box
