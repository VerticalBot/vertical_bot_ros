"""RoboDK-API-compatible Python package for VerticalBot Studio.

Drop-in replacement for the `robodk` package: `from robodk.robolink import *` and
`from robodk.robomath import *` work against a running VerticalBot Studio server
(ws://localhost:20500) instead of RoboDK. Scripts written for RoboDK run unchanged
for the common subset of the API (items, frames, targets, programs, moves, IO, posts).
"""
