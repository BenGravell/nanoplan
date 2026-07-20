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
Focus on getting planners performing well when no other actors are present.
We should see racelines (setting up outside of a corner and hitting the apex of the turn) emerge naturally. 


## Actor planning

--
Fix the actors. They should run a basic planner instead of using magic unphysical motion.

--
Add a left-menu tab with options for the non- ego racers.
1. Slider for the count. Should range from zero to eight.
2. Planner to use for actors.
3. Personality characteristics.
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
mouse scrolling with mouse outside of the driving canvas should not zoom the canvas

--
Make menu scale more proportionally with screen size, especially for large desktop screen sizes between 1440p and 2160p.

--
Speed gauge should always have a square bounding box with enough space for center number value text to not clip the gauge bar.

-- Friction box

Remove the +lon, +lat labels on grid.
Remove the numeric readout for lat and lon accel.

Put small accel bound values outside the grid next to axes lines. Express in gravity units i.e. +1.2g. Plus or minus for longitudinal bound limits, no sign for lateral bound units.

-- Add track lap stats.
Current elapsed time.
Previous lap time.
Best lap time so far.
Number of laps completed so far.

Display in the upper right corner.


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
