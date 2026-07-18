# TODO

--
Factor out the procedural track generator to a separate repo and crate.

Factor out the TUM racetrack database loader to a separate repo and crate.


--
For excitement and to make planning nontrivial:
Static obstacles - boulders and narrow track width.
Dynamic obstacles 
- other racers actors vehicles: trucks and motorcycle
- traverse 

--
Taxonomy of planners
- Sampling based (MPPI)
- Tree search (RRT)
- Local optimization (iLQR)

## basic planner

Fix the basic planners. Why do they fail to plan inside the road boundaries over the entire prediction horizon?

## Actor planning

Fix the actors. They should run a basic planner instead of using magic unphysical motion.

## collision physics

Fix the collision physics system. Actors and ego should bounce off each other, two way collisions. Treat every actor including ego as equal first class citizens in terms of collision physics. Road barriers are perfectly static and do not move, infinite inertia. 

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
Start menu
Put all shaded rectangle around the buttons on hover, and also on selection.

On selection, wiat just a short moment, perhaps 200ms, before proceeding, so that we get a visual indicator the button was pushed. Add an additional splash animation around the button to show it was selected.


--
Enforce minimum screen width and aspect ratio such that corner bkgd graphics never collide. add unit tests for that.

--
Exit button for mobile app needs to close app, hook into android and apple sys

--
Color the non-drivable area slightly darker grey than the road.

--
mobile controls
pinch to zoom

--
start the app fullscreen, both on mobile and desktop.

--
consider replacing the timeseries charts with a selector for coloring the ego carpet according to signals or metrics.
more fun and intuitive, and yields back space in the right rail.

--
ego carpet
rendering issues
1. flickering of one box about 4 boxes from the end
2. patches are not the correct thickness on mobile

--
Show the predicted future poses of actors in the viewer.
Re-use the ego carpet element.
Must stay lightweight on compute and rendering side.
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
