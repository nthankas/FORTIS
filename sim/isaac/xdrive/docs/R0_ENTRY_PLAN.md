# R0 Entry Deployment -- What We Know and What to Build

## The Path (Fixed, Non-Negotiable)
1. Robot drives horizontally through 15.75" access tunnel
2. Through R0 port (22" x 35.5" opening)
3. Front wheels go over the vessel wall lip at r≈103"
4. Robot pivots on the lip, tether controls descent
5. Front contacts interior slope (~35° from vertical)
6. Tether lowers robot down slope to flat floor
7. Robot reaches step-straddle position, tether releases, operation begins

## What Failed and Why
- Flat-bottom chassis catches on the lip edge -- chassis bottom hooks on sharp geometry
- PID tether with force application doesn't model a real cable -- a real cable can't have 0 tension when the robot is hanging off an edge
- The arch helps but motor mount flat areas still catch
- Manual control is impossible, simple automation doesn't handle the state transitions

## What Needs to Be Built in Isaac Sim

### Cable Model
- NOT a force-at-a-point. Model as a **distance constraint with a winch**.
- Cable has a current length. It can only get shorter (winch reels in) or longer (winch pays out).
- When the robot moves away from the anchor and the cable is taut, the cable pulls back -- automatically, like a real cable.
- Winch motor controls pay-out rate. PID on the winch controls descent speed.
- Cable attaches to rear of chassis via a Y-bridle (two attachment points on the rear corners).

### Chassis Design to Test
- 15" x 9" x variable height (test 3", 4", 5", 6")
- Trapezoidal arch on underbelly (test 1", 2", 3" arch heights)
- Wheels dropped below chassis (test 1", 1.5", 2", 2.5" drop)
- The winning config is whichever one clears the lip during the tether-controlled pivot
- Underbelly contact surfaces get small passive rollers or UHMW skid pads (no damage to graphite)

### Control System
State machine with IMU (pitch/roll) feedback:
- **TRAVERSE**: Wheels drive forward, cable trails with light tension (keep taut)
- **LIP_APPROACH**: Front wheels near edge, winch locks cable length, slow forward
- **PIVOT**: Pitch exceeds threshold, winch pays out at controlled rate, wheels stop driving
- **SLOPE_DESCENT**: Front wheels on slope, winch controls lowering speed
- **FLOOR_ARRIVAL**: Pitch returns to near-zero, winch releases to slack, wheels engage

### What to Measure
For each chassis config:
- Does it clear the lip during pivot? (underbelly contact = fail)
- Peak cable tension during pivot (sizes the winch motor)
- Peak wheel torque during slope descent
- Time to complete entry sequence
- Stability (does it tip sideways?)

## Masses to Sweep
- 30 lbs (13.6 kg) -- light config, no arm
- 40 lbs (18.1 kg) -- mid config
- 50 lbs (22.7 kg) -- heavy config with arm weight
