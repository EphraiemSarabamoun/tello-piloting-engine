# tello-piloting-engine

Closed-loop indoor drone navigation with a vision-language model in the perception loop.

A DJI Ryze Tello (the eighty-gram, hundred-dollar toy) flies a real spatial mission between
two colored Hue light beacons across an apartment. No GPS, no SLAM, no fiducial markers.
The drone takes off, flies to a magenta beacon in the kitchen, hovers, rotates 180 degrees,
and flies back to a cyan beacon in the bedroom. A small vision-language model running on a
local GPU answers one question per frame: is the goal beacon dominating the central third
of this view.

## Why

The interesting claim under the hood is that local VLMs are now fast enough and accurate
enough to be the perception layer for real-world control loops, on consumer hardware, in a
normal apartment. Moondream (1.7B parameters) warm-inferences in ~110 ms on an RTX 5090 at
480x360, and is 100% correct on the kitchen-frame goal-recognition task in the test set.
That puts perception bandwidth on the same order as the control loop frequency, which
changes what you can do with a tiny untethered drone.

## Architecture

The engine is split across three layers:

1. **`drone.py`** — CLI primitives over `djitellopy`: battery, snapshot, takeoff-land,
   first-flight, emergency cut-motors.
2. **`follow.py`** — face-follow autonomy. YuNet ONNX face detector with EMA-smoothed
   PID-style control on yaw, throttle, and pitch. Refuses takeoff under 20% battery,
   lands cleanly on SIGINT.
3. **`kitchen_simple.py`** — the full beacon-to-beacon mission. 20 Hz `send_rc_control`
   loop, 10 Hz snapshot dispatch, async VLM calls so the camera never blocks on
   inference, velocity ramps tied to HSV pixel counts (drone decelerates as the goal
   color saturates the frame), a 2-in-a-row goal streak to guard against single-frame
   noise, and a safety supervisor that lands on low battery or any unhandled exception.

Supporting library modules under `lib/`:

- `vlm_planner.py` — goal-conditioned planner using `gemma4:31b` via Ollama on the home GPU.
- `vlm_planner_claude.py` — fallback planner against the Anthropic API for when the local
  box is offline.
- `hue_beacon.py` — Hue v2 API client. Lights named after stars (Sirius, Vega, etc.).
  Magenta xy (0.41, 0.17) marks the kitchen, cyan xy (0.17, 0.30) marks the bedroom.
- `safety.py` — `safe_land` + SIGINT handler. RC zero, land, fall back to emergency.
- `pose.py` — odometry from RC commands plus IMU integration.
- `telemetry.py` — JSONL event log of every command, frame, decision, and state change.

## Hardware

- DJI Ryze Tello (regular, not EDU). Creates its own WiFi AP at `TELLO-XXXXXX`.
- A host with WiFi to associate with the drone AP. I fly from a Mac mini (`loki`) over
  wired Ethernet so the tailnet stays alive while WiFi joins the drone.
- An RTX 5090 (`napoleon`) running Ollama, exposed on `0.0.0.0:11434`. Serves
  `moondream:latest` for the kitchen mission and `gemma4:31b` for the general planner.
- Philips Hue bridge plus two color-capable bulbs. The mission depends on visible
  saturated color in the rooms it traverses.

## Run

```bash
uv sync
# Join the Tello WiFi AP first.
uv run python drone.py battery
uv run python drone.py first-flight        # 1m forward, 1m back, rotate 90, land
uv run python kitchen_simple.py            # full mission
uv run python follow.py 60                 # 60 seconds of face-follow
```

`kitchen_simple.py` accepts `--home Sirius` to pick which bedroom Hue serves as the home
beacon, and `--vlm-endpoint URL` to point at a different Ollama host.

## Safety

Indoor flight only. The drone weighs about 80 grams and drifts in any breeze. Every
mission script installs a SIGINT handler that lands cleanly; the supervisor watches
battery on every cycle and lands at 15%. If something goes wrong mid-air,
`uv run python drone.py emergency` cuts motors so the drone falls rather than flies into
a wall.

Tests under `tests/` cover the safety predicates, the pose math, the kitchen state
machine, the VLM planner's response parsing, and HSV color detection.

## Status

The face-follow and the kitchen mission both fly the routes they advertise on the rig
they were built on. The code is documented and tested. It is not a polished library:
constants are tuned for one apartment, one drone, one GPU. Read the source and tune
before flying.

## License

MIT.
