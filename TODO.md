# TODO

--
Factor out the procedurally track generator to a separate repo and crate.

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


Fix the basic planners. Why do they fail to plan inside the road boundaries over the entire prediction horizon?


Fix the actors. They should run a basic planner instead of using magic unphysical motion.

Fix the collision physics system. Actors and ego should bounce off each other, two way collisions. Treat every actor including ego as equal first class citizens in terms of collision physics. Road barriers are perfectly static and do not move, infinite inertia. 

Add guidance mode, human steers target for planner



## planning horizon

planning horizon might be too long, seems to cause bad behaviors like flickering and slowdown.

need to handle progress rewards somehow elegantly so that we encourage short-term acceleration without becoming too myopic and failing to reason about and anticipate corners and overtake maneuvers 



## road geometry - width vs curvature to avoid self intersection

Handle road widths that are too wide for curvature in a robust way. Must avoid self clipping of road geometry.

Limit local road width by the local radius of curvature on either side of the centerline, plus some small extra buffer to avoid degenerate near zero inner radius of road.

Use menger curvature to approximate radius of curvature from xy positions of centerline.

## road geometry - self intersection

Generated tracks sometimes intersect themselves. Not just a road width thing, but the centerline itself intersects. Road width also needs to be considered, i.e. actually materialize the full road geometry and check for self intersections of the full width road throughout the entire course. 


## Cost map

- Compute signed distance field to obstacles and road boundaries. Then take Euclidean distance transform to get a proximity cost map. This can be used for the collision and proximity costs and metrics.
This works for static obstacles.


## road model

https://github.com/BenGravell/nanoplan/tree/main/src/track

Save/check-in the trained model in repo.

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

Color the drivable area slightly darker grey than the background.

--
mobile controls
pinch to zoom

--
start the app fullscreen, both on mobile and desktop.

--
consider replacing the timeseries charts with a selector for coloring the ego carpet according to signals or metrics. more fun and intuitive, and yields back space in the right rail.

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
Trajectory tree: two levels of hierarchy of branching, very thick edge weights, inside a steering wheel toncreate a disk lockup.

--
Add timescrubbers for freezing simulation and replaying past.

Make the timescrubbers bigger with big touch targets for the grab handles.
Use the full page width.
Put the at the bottom of the screen in dedicated area/container like a video player would have.

--
Give the various actors in the scene minimal meshes representative of the semantic class (car, truck, cycle, pedestrian, etc)
