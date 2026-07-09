# TODO

## Business logic and design

--
Factor out shared types and helpers across planners in src/planning

--
src/planning/treetop/rrt.rs
ACCEL_LAT_MAX is referenced but does not exist


--
Factor out magic numbers from
src/planning/treetop/mod.rs
And especially the ones that are shared across other planners - single source of truth. 

--
Unify all quasi Monte Carlo sampling to use the same sequences and source code. Halton, van der corput, sobol

--
Integrate interfaces of judo more thoroughly. Reuse sampling.

--
Re-tune the expert human cost weights using nuPlan.

-- 
Use the expert human tuned cost weights by default.


## FEATURES

--
Make the real CommonRoad scenarios available. Vendor them.


## UX

--
UX
Give nanoplan a unique icon/favicon.
Display the favicon on the website app ( browser tab), both local and cloud deployed.
Use a Combination of ai generation and open source iconoggrsphy according to Best design principles a la Allan Peters.

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
Give the various actors in the scene minimal polyhedral 3d meshes representative of the semantic class (car, truck, cycle, pedestrian, etc)

Add basic ultra fast 3D rendering along with 3d camera controls.
First and third person ego follower cameras.
