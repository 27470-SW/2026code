# Road Runner tuning OpModes (RR 0.5.4)

These are the Road Runner 0.x quickstart tuners, adapted to `MecanumDriveLRR`, the Pinpoint
localizer, and `RobotConstants`. They are in the same app as the competition OpModes: flash
once, then pick them from the Driver Station (group **RR Tuning**, names start with `RR`).

Only the drivetrain is created, so mechanisms don't move. Open FTC Dashboard at
`http://192.168.43.1:8080/dash` to graph values and edit constants live.

## Order

| # | OpMode | What you set |
|---|--------|--------------|
| 1 | `RR 1 LocalizationTest` (TeleOp) | Pinpoint offsets/directions in `PinpointLocalizer` until x/y/heading are right |
| 2a | `RR 2a MaxVelocityTuner` | `MAX_VEL` to ≤ 80% of the result |
| 2b | `RR 2b MaxAngularVeloTuner` | `MAX_ANG_VEL` to ≤ 80% of the result |
| 3a | `RR 3a AutomaticFeedforwardTuner` | starting `kV`, `kStatic`, `kA` |
| 3b | `RR 3b ManualFeedforwardTuner` | refine `kV`/`kA`/`kStatic` (AXIS=FORWARD), then `LATERAL_MULTIPLIER` (AXIS=STRAFE) |
| 4 | `RR 4 TrackWidthTuner` | `DT_TRACK_WIDTH` |
| 5 | `RR 5a/5b/5c Straight/Strafe/TurnTest` | sanity checks |
| 6 | `RR 6a BackAndForth`, `RR 6b FollowerPIDTuner` | `TRANSLATIONAL_PID`, `HEADING_PID` |
| 7 | `RR 7 SplineTest` | final check |

`DriveVelocityPIDTuner` is not included because `RUN_USING_ENCODER` is false (feedforward mode).

## Saving the results

Dashboard edits are **not saved**. They last until the Robot Controller app restarts, or until an
OpMode that calls `RobotConstants.init()` is initialized (**Decode Auton** does), which puts the
code values back. Write the final numbers into `RobotConstants.init()` (the chassis section) and
re-flash once at the end.

Use a charged battery: `MecanumDriveLRR` does not voltage-compensate its feedforward.

## How this maps to Road Runner 1.0 tuning

| RR 1.0 step | Here |
|---|---|
| ForwardPushTest / LateralPushTest (inPerTick) | Not needed: Pinpoint gives inches directly (LocalizationTest) |
| ForwardRampLogger (kS, kV) | AutomaticFeedforwardTuner |
| LateralRampLogger (lateralInPerTick) | ManualFeedforwardTuner with AXIS=STRAFE → LATERAL_MULTIPLIER |
| AngularRampLogger (trackWidthTicks) | TrackWidthTuner |
| ManualFeedforwardTuner (kV, kA) | ManualFeedforwardTuner |
| ManualFeedbackTuner (axial/lateral/heading gains) | BackAndForth / FollowerPIDTuner → TRANSLATIONAL_PID / HEADING_PID kP (and kD) |
| SplineTest | SplineTest |
