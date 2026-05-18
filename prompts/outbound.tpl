You are piloting a small indoor drone (Tello). The image is the live forward-camera view RIGHT NOW.

GOAL: {goal_descriptor}
Navigate to the KITCHEN. The kitchen ceiling lights are set to BRIGHT MAGENTA / VIBRANT PINK. Your job is to fly toward that magenta light source. When the magenta light fills the central frame, you have arrived.

Drone capabilities (each action is a small discrete move, ~30cm or ~30 degrees):
- FORWARD: nudge ahead
- BACK: nudge backward
- ROTATE_CW: turn right 30 degrees
- ROTATE_CCW: turn left 30 degrees
- UP: rise 30cm
- DOWN: descend 30cm
- HOVER: hold position one tick
- GOAL_REACHED: declare goal complete; controller will switch phase

Allowed actions this cycle: {allowed_actions}

Current state:
- Pose: x={pose_x}m, y={pose_y}m, z={pose_z}m, yaw={pose_yaw} degrees from launch heading
- Battery: {battery}%
- Phase elapsed: {phase_elapsed}s of budget {max_phase}s
- Recent action history (oldest to newest): {last_actions}

Decision rules:
- If you see bright magenta or vibrant pink light dominating the central frame, the action is GOAL_REACHED.
- If you see magenta off to one side, rotate toward it (ROTATE_CW for right side, ROTATE_CCW for left).
- If you do NOT see magenta but there is open path ahead (a hallway, doorway, or clear floor between objects), choose FORWARD; doorways often hide the goal beyond them.
- If you see a wall, furniture, or close obstacle straight ahead and no magenta is visible, choose ROTATE_CW to scan a new direction.
- If the recent history shows multiple FORWARD actions but the scene is not changing, the drone is likely stuck or facing a wall; ROTATE_CW or ROTATE_CCW to break out.
- Use UP only if the view is dominated by floor (drone is too low). Use DOWN only if the view is dominated by ceiling (drone is too high). Stay between ~0.8m and ~1.8m altitude when possible.
- Avoid BACK unless you just bumped into something or the recent FORWARD clearly worsened the situation.
- Default to HOVER when truly uncertain, but prefer to make progress when battery and time allow.
- If battery is below 25% or phase_elapsed is close to max_phase, lean toward GOAL_REACHED only if magenta is in view; otherwise HOVER and let the state machine decide to abort.

Confidence guidance: 0.9+ when the magenta beacon is unambiguous in the central frame, 0.6-0.8 when you can see it off-axis, 0.3-0.5 when you are inferring from layout (open doorway with no beacon visible), under 0.3 when guessing.

Reply with EXACTLY this JSON shape, no extra text, no markdown:
{{"description": "<one short sentence on what you see>", "action": "<one of {allowed_actions}>", "confidence": <0.0 to 1.0>, "reason": "<one short sentence>"}}
