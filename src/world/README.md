# Live driving world

`LiveWorld` is the realtime demo loop.
`Track` supplies centerline points, headings, and widths at any longitudinal
progress by wrapping a generated or downloaded closed circuit across laps.

Every tick rebuilds a short, coarse planning window when needed, advances the single-track traffic with kinematic gap control, passes only actors whose reachable interval can overlap the ego's, calls the selected planner, and applies its first control. Cars that fall far behind are recycled ahead.
