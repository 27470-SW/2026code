# Road Runner tuning (RR 0.5.4 + Pinpoint)

These OpModes are in the same app as the competition code: flash once, then pick them from the
Driver Station (group **RR Tuning**, names start with `RR`). Only the drivetrain is created, so
mechanisms don't move.

Open FTC Dashboard at `http://192.168.43.1:8080/dash`. Every value below is under
**RobotConstants** (or the OpMode's own name) in the Configuration panel and can be changed live.

## The big idea: three layers, tuned bottom-up

| Layer | Job | Parameters |
|---|---|---|
| **Localization** (Pinpoint) | Know where the robot *is* | Pinpoint pod type, directions, offsets |
| **Feedforward** | Guess the motor power needed to follow the path, *without looking* | `kV`, `kStatic`, `kA`, `LATERAL_MULTIPLIER`, `DT_TRACK_WIDTH` |
| **Feedback (PID)** | Look at the error (where it should be minus where it is) and correct it | `TRANSLATIONAL_PID`, `HEADING_PID` |

**Yes, the PIDs still matter with a Pinpoint.** The Pinpoint only *measures* position. The PIDs are
what *act* on that measurement. Without them the robot runs on feedforward alone and drifts. The
Pinpoint makes PID tuning *easier*: it doesn't slip like drive-wheel encoders, so the error the PID
sees is the real error.

**Why the old back-and-forth was so painful:** if the feedforward is off, the PID has to make up
for it on every move. You end up with big gains that oscillate, or kI that winds up. The fix
(and the main idea behind RR 1.0's tuning) is to get feedforward right first, with the PIDs
turned **off**, so the PID only has small errors left to correct. Then each feedforward parameter
has one test, and only the two PIDs need iterating.

## Step by step

Use a charged battery the whole time: the feedforward is not voltage-compensated.

### 1. Localization: `RR 1 LocalizationTest` (TeleOp)
Drive with the left stick, turn with the right stick.

| Check | If wrong, change (in `PinpointLocalizer`) |
|---|---|
| Push forward → **x goes up** | Flip the X pod's `EncoderDirection` |
| Push left → **y goes up** | Flip the Y pod's `EncoderDirection` |
| Turn CCW → **heading goes up** | Pinpoint mounting / orientation |
| Spin in place several times → x and y stay near 0 | Pod offsets (`setOffsets`) |
| Face 90°, drive forward → **robot vel x** is positive and robot vel y is ~0 | Set `PinpointLocalizer.VELOCITY_TO_ROBOT_FRAME` false |
| Drive straight along 4 field tiles (96 in) → x reads roughly 96 | Pod type (`setEncoderResolution`) |

The last row is only a rough check (a wrong pod type is ~50% off, easy to see). The exact
tape-measure check is the first thing in step 3.

This has to come before step 2: the max velocity tuners read their speeds from the Pinpoint.

### 2. Speed limits: `RR 2a MaxVelocityTuner`, `RR 2b MaxAngularVeloTuner`
Set `MAX_VEL` and `MAX_ANG_VEL` (radians) to about **80%** of the measured maximum.
A path can't ask for more speed than the motors have.

This has to come before step 3: StraightTest's path is planned at `MAX_VEL`. With a `MAX_VEL` the
robot can't reach, it always comes up short and step 3 would suggest a `kV` that's too big.

### 3. Drive forward and feedforward: `RR 5a StraightTest`, then `kV`, `kStatic`, `kA`
**StraightTest** drives itself forward `DISTANCE` = 100 in (no joystick), with `FEEDBACK = false`.
Every run gives you two numbers to compare:

1. **Tape measure vs. finalX → Pinpoint accuracy.** Measure how far the robot really went and
   compare it to the **finalX** it reports. They should agree within about ½ in, whether or not
   it stopped at 100. If not, fix the pod type / directions and redo step 2.
2. **finalX vs. 100 → `kV`.** Came up short → raise kV. Went too far → lower kV. It prints a
   suggested kV. Enter it in Dashboard and run again until it stops within an inch or two.

To fine-tune past that (or fill in `kStatic` and `kA`):
* **`RR 3a AutomaticFeedforwardTuner`** gives starting values for all three (Y = fit kStatic,
  Y = fit kA). Optional if StraightTest already got kV close.
* **`RR 3b ManualFeedforwardTuner`** (AXIS = FORWARD) drives back and forth with the PIDs not
  involved. Graph `targetVelocity` and `measuredVelocity` in Dashboard:

| What the graph shows | Change |
|---|---|
| Flat (cruising) part: measured **below** target | Raise `kV` |
| Flat part: measured **above** target | Lower `kV` |
| Measured lags at the very start / near zero speed | Raise `kStatic` |
| Ramps (speeding up / slowing down) lag behind target | Raise `kA` |
| Ramps overshoot target | Lower `kA` |

Get kV right first, then kStatic, then kA.

### 4. Strafe: `LATERAL_MULTIPLIER` (your old "strafe" test)
`RR 5b StrafeTest` with `FEEDBACK = false`. Mecanum wheels slip when strafing, so a strafe needs
more power than driving straight; `LATERAL_MULTIPLIER` is that extra.
Came up short → raise it. Went too far → lower it. It prints a suggested value.
Stop and restart the OpMode after changing it (it's read when the drive is created).

With the Pinpoint, `LATERAL_MULTIPLIER` no longer affects *where the robot thinks it is*. It only
affects how hard the robot *pushes* when strafing.

Alternative with a graph: `RR 3b ManualFeedforwardTuner` with AXIS = STRAFE. Adjust until
measured matches target.

### 5. Turning: `DT_TRACK_WIDTH`
`RR 4 TrackWidthTuner` turns 5 times with heading feedback off and prints the value to use.
`RR 5c TurnTest` with `FEEDBACK = false` is a quick single-turn version.
Under-turned → raise `DT_TRACK_WIDTH`. Over-turned → lower it.
(This is an "effective" track width for turning power. It won't match a tape measure exactly,
and that's fine.)

### 6. Feedback: `TRANSLATIONAL_PID`, `HEADING_PID` (back-and-forth and square tests)
Now the PIDs only have small errors to correct. Changes in Dashboard apply immediately.

**`RR 6a BackAndForth`** → `TRANSLATIONAL_PID`. Graph `xError`.
1. Set kI = 0 and kD = 0.
2. Raise **kP** until xError stays small (well under 1 in) during the move.
3. If it overshoots or wobbles at the end of each move, add a little **kD** (start around kP/20).
   If it still oscillates, lower kP.
4. Leave **kI at 0** unless the robot always stops a little short and kP can't fix it. kI is what
   usually made this step painful.

**`RR 6b FollowerPIDTuner`** (the square) → `HEADING_PID`, and also checks translational.
Graph `headingError (deg)` and watch the field view.
Same recipe: kP up until heading error stays within ~1–2°, a little kD if it wobbles, kI = 0.

`TRANSLATIONAL_PID` is used for both forward and sideways, so also check `yError` here (the
square includes moves after turning).

This is the only step where you iterate between two things. If heading wobbles during straight
moves, fix heading first and then recheck translational.

### 7. Final checks
* `RR 5a/5b/5c` with **`FEEDBACK = true`**: each should end within ~0.5 in / ~1°, both by the
  reported pose and by tape measure.
* `RR 7 SplineTest`: the robot should follow the drawn path in the Dashboard field view.

## Saving results

Dashboard edits are **not saved**. They last until the Robot Controller app restarts, or until an
OpMode that calls `RobotConstants.init()` is initialized (**Decode Auton** does), which puts the
code values back. Write each value down as you go. At the end, put them in `RobotConstants.init()`
(the chassis section, where kV/kA/etc. are set today) and flash once.

## Quick reference: parameter → test

| Parameter | Test | Setting |
|---|---|---|
| Pinpoint directions / offsets / pod type | RR 1 LocalizationTest | `PinpointLocalizer` |
| `MAX_VEL` | RR 2a MaxVelocityTuner | 80% of result |
| `MAX_ANG_VEL` | RR 2b MaxAngularVeloTuner | 80% of result |
| Pinpoint accuracy (exact) | RR 5a StraightTest | tape vs. finalX |
| `kV`, `kStatic`, `kA` | RR 5a (FEEDBACK=false), then RR 3a / RR 3b (FORWARD) | |
| `LATERAL_MULTIPLIER` | RR 5b (FEEDBACK=false) or RR 3b (STRAFE) | |
| `DT_TRACK_WIDTH` | RR 4 TrackWidthTuner or RR 5c (FEEDBACK=false) | |
| `TRANSLATIONAL_PID` | RR 6a BackAndForth | graph xError |
| `HEADING_PID` | RR 6b FollowerPIDTuner | graph headingError |

`DriveVelocityPIDTuner` / `MOTOR_VELO_PID` is not used: `RUN_USING_ENCODER` is false.

## How this maps to Road Runner 1.0 tuning

| RR 1.0 step | Here |
|---|---|
| ForwardPushTest / LateralPushTest | Not needed: the Pinpoint reports inches directly (step 1) |
| ForwardRampLogger (kS, kV) | AutomaticFeedforwardTuner |
| LateralRampLogger | StrafeTest (FEEDBACK=false) / ManualFeedforwardTuner STRAFE |
| AngularRampLogger | TrackWidthTuner |
| ManualFeedforwardTuner | ManualFeedforwardTuner |
| ManualFeedbackTuner (still a back-and-forth) | BackAndForth / FollowerPIDTuner |
| SplineTest | SplineTest |
