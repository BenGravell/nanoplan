# TODO


--
Strip down

What brings you joy?
Leave only that

Procedurally generated world and actors.

Seeing ego plan agile maneuvers - a strong planner
Seeing ego plan crazy silly or exploitation maneuvers

Get rid of the road network. Too much lift to get lanes and connecivity.
Use an infinite racetrack. 
Then we eliminate the route planner.
Can still be procedurally generated but just have a single lane with varying width and curvature.

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


## Ego carpet
- Take the locus of all ego footprint. Use a finer time grid than 0.1s to ensure spatial overlap at high speed.
Slice the locus into oriented rectangular bands.
Color each band by the time it is occupied (occupancy is actually an interval, so choose min or max or avg).

## Cost map.
- Compute signed distance field to obstacles and road boundaries. Then take Euclidean distance transform to get a proximity cost map. This can be used for the collision and proximity costs and metrics.
This works for static obstacles.


--
Re-tune the expert human cost weights using nuPlan.

-- 
Use the expert human tuned cost weights by default.


## UX


-- 
camera controls in scenarios mode

--
enhanced camera controls - pan, rotate, follow/detach from ego, reset

--
UX
Give nanoplan a unique icon/favicon.
Display the favicon on the website app ( browser tab), both local and cloud deployed.
Use a combination of AI generation and open source iconography according to Best design principles a la Allan Peters.

Symbology:
Trajectory tree: two levels of hierarchy of branching, very thick edge weights, inside a steering wheel toncreate a disk lockup.


-- 
UX
Use bigger gui elements for the controls, they are tiny currently.


--
UX
Use fonts
Atkinson Hyperlegible Next (body)
Atkinson Hyperlegible Mono code, numeric values)
Space Grotesk (headers)

--
UX
Revise the user controls menu.
Use proper hierarchy.
Separate out the sections for
Scenario selection
Planner selection + config
Viewer visibility settings
Metrics
Diagnostics

Take inspiration from racing video games and fighter pilot HUD.

--
UX
Add camera controls:
Zoom
Pan
Follow Ego (position, yaw)
Reset


--
UX
Make the timescrubbers bigger with big touch targets for the grab handles.
Use the full page width.
Put the at the bottom of the screen in dedicated area/container like a video player would have.



--
UX
Add timeseries charts in a column rail along the right edge of the screen:
Speed
Longitudinal acceleration 
Lateral acceleration
Curvature

The x axis (time) should be hard synchronized between all plots.

Show trace for actual Ego in thick white line, trace for planned trajectory in thinner line matching the accent color for semantic "planned" meaning (pink). Link all the semantic meaning colors with a single source of truth color definition.



--
UX
Imbue the app with a y2k metal heart aesthetic, but keep the modern ultra minimal clean core.
App should feel like a mix of Wipeout and Gran Turismo 3 racers.

-- 
Draw ego future carpet. Carpet represents the region of space that will be occupied by the ego at every point in the future over the planning horizon/duration of trajectory. It is the ego footprint at every tick over entire future trajectory, spatially merged and resulting polygon simplified.

--
Give the various actors in the scene minimal meshes representative of the semantic class (car, truck, cycle, pedestrian, etc)
