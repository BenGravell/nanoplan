# Live driving world

`LiveWorld` is the realtime demo loop.
`Track` supplies centerline points, headings, and widths at any longitudinal
progress by wrapping a generated or downloaded closed circuit across laps.

Every tick rebuilds a short, coarse planning window when needed, advances the single-track traffic with gap control, passes only actors whose reachable interval can overlap the ego's, calls the selected planner, and applies its first control. Ego and traffic then enter the same collision solve; road barriers are static. Collision velocity carries into later traffic ticks rather than being replaced by lane following. Actors retain continuous progress around the closed circuit; distant actors are culled from planning rather than relocated.
