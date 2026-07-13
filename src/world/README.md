# Endless track world

`LiveWorld` is the realtime demo loop. `Track` supplies deterministic
centerline points, headings, and widths at any longitudinal coordinate, so the
world has no end and needs no graph or chunk loader.

Every tick rebuilds a short, coarse planning window when needed, advances the
single-track traffic with IDM, passes only actors whose reachable interval can
overlap the ego's, calls the selected planner, and applies its first control.
Cars that fall far behind are recycled ahead.
