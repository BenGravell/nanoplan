# Live driving world

`LiveWorld` is the realtime demo loop.
`Track` supplies centerline points, headings, and widths at any longitudinal progress by wrapping a checked-in closed
circuit across laps.

Every tick rebuilds a short, coarse planning window when needed, advances the single-track traffic with gap control,
passes only actors whose reachable interval can overlap the ego's, calls the selected planner, and applies its first
control.
All planners receive the same road-window policy: maximum-acceleration reach over the 10-second planning horizon,
including drag, plus 25 m ahead and 50 m behind.
The window refreshes after 20 m of progress or sooner when increasing speed needs more road.
Planner creation and switching use this same policy.
Ego and traffic then enter the same collision solve; road barriers are static.
Collision velocity carries into later traffic ticks rather than being replaced by lane following.
Actors retain continuous progress around the closed circuit; distant actors are culled from planning rather than
relocated.
