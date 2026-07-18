# TODO

--
Factor out the procedural track generator to a separate repo and crate.

Factor out the TUM racetrack database loader to a separate repo and crate.

--
Taxonomy of planners
- Sampling based (MPPI)
- Tree search (RRT)
- Local optimization (iLQR)

--
src/prediction.rs
Notion of lane and lane association are irrelevant on a race track. Remove lane association logic.

--
src/prediction.rs
Prediction model should assume that actors will initially follow the track at the actors current lateral offset from centerline, then return to track centerline over time.
Basically use a Frenet motion decomposition.
Longitudinal motion: maintain current speed
Lateral motion: smooth return to centerline

--
Focus on getting planners performing well when no other actors are present.
We should see racelines (setting up outside of a corner and hitting the apex of the turn) emerge naturally. 


## Actor planning

--
Fix the actors. They should run a basic planner instead of using magic unphysical motion.

--
Add options for the non- ego racers.
1. Slider for the count. Should range from zero to eight.
2. Planner to use for actors.
3. Personality characteristics. 
  - Assertiveness - progress weight
  - Recklessness - safety weight

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
