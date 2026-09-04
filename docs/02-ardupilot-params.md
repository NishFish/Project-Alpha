# ArduPilot parameters for obstacle avoidance

Set these in Mission Planner under Config -> Full Parameter List. They work the
same in SITL and on the real board.

## Turn it on

| Parameter | Set to | What it does |
|---|---|---|
| `PRX1_TYPE` | `2` | Accept proximity data over MAVLink. Older firmware calls this `PRX_TYPE`. |
| `PRX1_ORIENT` | `0` | Sensor is mounted forward, right way up. |
| `OA_TYPE` | `1` | BendyRuler path planning in AUTO and Guided. `2` is Dijkstra. |
| `OA_MARGIN_MAX` | `2` | Stay 2 m off obstacles. |
| `AVOID_ENABLE` | `7` | Simple stop-short avoidance in Loiter/PosHold. |

**`OA_TYPE` is the one that matters.** If it does not exist in your parameter
list, your firmware build has object avoidance compiled out. Run
`src/check_avoidance_support.py` to check this properly.

## Keep it slow

| Parameter | Set to | Why |
|---|---|---|
| `WPNAV_SPEED` | `200` | 2 m/s in AUTO (units are cm/s). |
| `WPNAV_ACCEL` | `100` | Gentle acceleration. |

2 m/s is not a placeholder. It is derived from how far the camera can see
reliably versus how long the drone takes to stop. Raise it only after Step 8
passes cleanly, and raise it gradually.

## Companion computer link (Step 7, on the real board)

Wire the Pi to the Pixhawk's **TELEM2** port.

| Parameter | Set to | Why |
|---|---|---|
| `SERIAL2_PROTOCOL` | `2` | MAVLink 2 |
| `SERIAL2_BAUD` | `460` | 460800. Clone boards are often unreliable at 921600 -- start here. |

## Safety

| Parameter | Notes |
|---|---|
| `FENCE_ENABLE` | Turn it on. Set a small radius and a low altitude for early tests. |
| `FENCE_RADIUS` | Start at 50 m. |
| `FENCE_ALT_MAX` | Start at 10 m. |

Remember the failure mode: if the Pi stops sending, ArduPilot discards the stale
data and **carries on with no avoidance**. The geofence and the speed cap are
what protect you when that happens, so do not treat them as optional.
