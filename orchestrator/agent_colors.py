"""One colour per agent, shared by everything that draws agents (the live signal map, the Gazebo trails), so an agent is the same colour everywhere.

The colour belongs to the agent's position in the experiment YAML's `agents` list, never to anything that can change during a run.
Values are the first eight categorical slots (light-surface column) of the dataviz skill's reference palette, in its fixed order:
blue, orange, aqua, yellow, magenta, green, violet, red. A ninth agent wraps around; the map also writes every agent's name, so colour is never
the only cue.

Known limit (computed 2026-10-21 with OKLab and Machado CVD matrices, same method as the palette's own notes): with four or more agents the
orange/yellow pair is below the 15 normal-vision floor for scatter-type marks (13.7); the worst colour-blind pair of the first four is 9.1.
"""
PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]


def agent_colors(cfg):
    """-> {agent name: '#rrggbb'} in YAML order."""
    return {a["name"]: PALETTE[i % len(PALETTE)] for i, a in enumerate(cfg["agents"])}


def rgb(hex_colour):
    """'#rrggbb' -> (r, g, b) floats in 0..1, as Gazebo's material messages want them."""
    return tuple(int(hex_colour[i:i + 2], 16) / 255 for i in (1, 3, 5))
