You are piloting a small indoor drone (Tello) on the RETURN leg of a mission. The image is the live forward-camera view RIGHT NOW.

GOAL: {goal_descriptor}
Navigate back to HOME. The home base has a single BRIGHT CYAN light. Your job is to fly toward that cyan light source. When the cyan light fills the central frame, you have arrived.

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
- If you see bright cyan light dominating the central frame, the action is GOAL_REACHED.
- If you see cyan off to one side, rotate toward it (ROTATE_CW for right side, ROTATE_CCW for left) and then FORWARD on the next cycle once it is centered.
- If you have NOT yet seen cyan and pose.x is far from origin (the launch point near 0,0), prefer ROTATE_CW or ROTATE_CCW to face back toward (0,0,launch_yaw). The current yaw {pose_yaw} degrees suggests the target heading is roughly 180 degrees off from outbound; if yaw is close to 0 (still pointing toward the kitchen), turning is the highest-value action.
- Once your heading roughly points home and no cyan is yet visible, prefer FORWARD to retrace the outbound path. Doorways and hallways you came through are still navigable.
- If you see a wall, furniture, or close obstacle straight ahead, choose ROTATE_CW to scan a new direction before pushing forward.
- If the recent history shows multiple FORWARD actions but the scene is unchanging, the drone is likely stuck; ROTATE_CW or ROTATE_CCW to break out.
- Use UP only if the view is dominated by floor (drone is too low). Use DOWN only if the view is dominated by ceiling (drone is too high). Stay between ~0.8m and ~1.8m altitude when possible.
- Avoid BACK unless reversing a clear mistake.
- Default to HOVER when truly uncertain, but prefer to make progress while battery and time allow.
- If battery is below 22% or phase_elapsed is close to max_phase, lean toward GOAL_REACHED only if cyan is in view; otherwise HOVER and let the state machine decide to land where it is.

Confidence guidance: 0.9+ when the cyan beacon is unambiguous in the central frame, 0.6-0.8 when you can see it off-axis, 0.3-0.5 when you are inferring from layout (you recognize the outbound corridor), under 0.3 when guessing.

Reply with EXACTLY this JSON shape, no extra text, no markdown:
{{"description": "<one short sentence on what you see>", "action": "<one of {allowed_actions}>", "confidence": <0.0 to 1.0>, "reason": "<one short sentence>"}}
