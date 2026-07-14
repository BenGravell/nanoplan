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

## Cost map

- Compute signed distance field to obstacles and road boundaries. Then take Euclidean distance transform to get a proximity cost map. This can be used for the collision and proximity costs and metrics.
This works for static obstacles.

## UX

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
Add timeseries charts in a column rail along the right edge of the screen:
Speed
Longitudinal acceleration 
Lateral acceleration
Curvature

The x axis (time) should be hard synchronized between all plots.

Show trace for actual Ego in thick white line, trace for planned trajectory in thinner line matching the accent color for semantic "planned" meaning (pink). Link all the semantic meaning colors with a single source of truth color definition.

--
Give the various actors in the scene minimal meshes representative of the semantic class (car, truck, cycle, pedestrian, etc)
