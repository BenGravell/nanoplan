# TODO

--
Strip down

What brings you joy?
Leave only that.

Procedurally generated world and actors.

Seeing ego plan agile maneuvers - a strong planner
Seeing ego plan crazy silly or exploitation maneuvers

For excitement and to make planning nontrivial:
Static obstacles - boulders and narrow track width.
Dynamic obstacles 
- other racers actors vehicles: trucks and motorcycle
- traverse 

Need for Speed: Hot Pursuit
- Police car chaser. If they catch you, you lose.

Taxonomy of planners
- Sampling based (MPPI)
- Tree search (RRT)
- Local optimization (iLQR)

## planning horizon

planning horizon might be too long, seems to cause bad behaviors like flickering and slowdown.

need to handle progress rewards somehow elegantly so that we encourage short-term acceleration without becoming too myopic and failing to reason about and anticipate corners and overtake maneuvers 


## Cost map

- Compute signed distance field to obstacles and road boundaries. Then take Euclidean distance transform to get a proximity cost map. This can be used for the collision and proximity costs and metrics.
This works for static obstacles.

## UX

--
mobile controls
pinch to zoom

--
start the app fullscreen

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
