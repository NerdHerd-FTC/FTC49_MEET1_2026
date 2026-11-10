# Shooting and autonomous setup

Distances use Pedro field inches (X/Y 0–144). Panels angles are degrees; Pedro Pose headings are radians.
All live settings are in `Subsystems/Scoring/ShootingConfig.java`.

## Controls

- Operator LB selects LEFT, RB selects RIGHT, from their own alliance station looking onto the field. Selection persists; default is RIGHT.
- Turret: the motor turret always points at the selected target from odometry. Every loop it recalculates the robot-relative target angle from the current pose, then sets power = PID(target − measured angle). No button, and the drivetrain is never turned to aim.
- Wrap-around: of every equivalent target (angle ± 360°) inside `turretMinDeg..turretMaxDeg`, the turret goes to the one nearest its current angle, never past a limit. If none fits, it stops at the closest limit and telemetry says to turn the robot.
- Operator LT: any press feeds immediately; no readiness checks. Telemetry "Turret on target" shows whether the turret is within `turretToleranceDeg` of its command.
- Turret start: turret angle = `turretStartDeg` + encoder ticks / `turretTicksPerDeg`. Put the turret at `turretStartDeg` before INIT of any auto; TeleOp keeps that count. If TeleOp runs without an auto first in this app session, put it there before TeleOp INIT.
- Driver X: reverse intake/feeder and keep manual driving. X wins over LT.
- Driver LB: half-speed manual driving.
- Driver Y: set the current X/Y/heading to that alliance's configured corner pose, once per press. Place the robot at the measured corner position and facing before pressing.
- Shooter speed follows distance continuously, including while driving, aiming, reversing or being pushed. Front intake runs continuously except when reversed.
- Release LT: feeder stops.
- Both TeleOps use the same Pedro field-centric axes and current global heading, with no alliance joystick offset.

## Configuration

| Setting | Meaning |
|---|---|
| `redStartXIn/YIn/HeadingDeg`, `blueStartXIn/YIn/HeadingDeg` | Actual auto starting placement; also TeleOp fallback if no auto pose was saved. Set before INIT. |
| `redResetXIn/YIn/HeadingDeg`, `blueResetXIn/YIn/HeadingDeg` | Driver Y corner-reset pose, independent per alliance and editable live in Panels. All six defaults are zero placeholders; measure Pedro's robot reference point at the corner and its heading before using. |
| `shooterForwardIn`, `shooterLeftIn` | Turret pivot relative to Pedro's robot reference point. Forward/left are positive. |
| `shooterYawOffsetDeg` | Shooter direction at turret angle 0 relative to robot forward; CCW positive. |
| `turretStartDeg` | Physical turret angle at INIT (0 forward, 90 left). |
| `turretTicksPerDeg` | Measure with TURRET TUNER. Placeholder 1.494 (537.7 ticks/rev, 1:1). |
| `turretReversed` | Flip if positive power turns the turret clockwise (TURRET TUNER shows negative ticks for a CCW turn). |
| `turretMinDeg`, `turretMaxDeg` | Safe travel limits, robot-relative, CCW positive; default ±180. Up to 720° total; wider than 360° (slip ring/slack) gives true wrap-around. |
| `turretKp/Ki/Kd` | Turret PID on angle error in degrees → power; defaults 0.02 / 0 / 0.001. |
| `turretMaxPower` | Turret power cap; default 0.6. |
| `turretToleranceDeg` | Turret "on target" telemetry window; default 2°. |
| Four `red/blueLeft/RightXIn/YIn` pairs | Aim-point coordinates for each alliance/cell. |
| Four `...Shots` strings | Measured distance:velocity pairs, e.g. `10:500,50:900,100:1200`, inches:ticks/second. Distances increase; speeds are positive. |
| Four `...Calibrated` flags | Telemetry reminders only; false does not block feeding. |
| `kp/ki/kd/kf` | Live shooter PIDF; changed valid values are sent to the motor controller. |

Starting poses, shooter offsets and shot tables are placeholders. The shipped `70:950,76.05:1000` tables reproduce two points on the old `origin/Shooting_Formula` equation; they are not measured shots. Measure shots at several distances and approach angles for each cell, then update the tables. Record physical release height and launch angle in your calibration notes; changing them requires recalibration. They are not live controls because this code uses measured speed tables.

Outside a table's range, speed clamps to its nearest endpoint and telemetry warns. Invalid tables or non-finite geometry command zero speed. There is no speed cap: keep table speeds within what the flywheel can reach. LT always feeds, using the same distance-derived speed; it cannot reconstruct distance from broken odometry. On invalid geometry the turret brakes in place. Invalid PIDF edits retain the last valid gains while speed updates continue.

## Code flow

- `RedTeleop.java` / `BlueTeleop.java`: initialize from their own saved pose, read bumper selection and select their own target/table.
- `MyRobot.update()`: update odometry, calculate the shot, apply live PIDF and speed, aim the turret, then handle X → LT.
- `ShotTargeting.java`: plain geometry/interpolation helper. Warning text does not control feeding.
- `DriveSubsystem.java`: Pedro field-centric manual drive only.
- `TurretSubsystem.java` / `AimTurretCommand.java`: pick the wrap-around target (`ShotTargeting.nearestTurretAngle`) and run one PID step on `TurretMotor` each loop. The same odometry sample determines turret angle and speed.
- `TurretTuner.java` (TURRET TUNER): during INIT the motor floats; turn the turret exactly 90° CCW by hand and read ticks/deg. After START, dpad left/up/right/down = 90/0/−90/180° to tune PID live.
- `ScoringSubsystem.java`: motor velocity and live PIDF. Speed is set directly each loop. Autos set their own speed per shot with `setVelocity()`, e.g. AutoPath1's `shootStart(950)`.
- STOP uses the normal FTC loop exit and directly stops drive, shooter and intake outputs.

## Auto templates and pose handoff

`RedAuto.java` registers RED AUTO and preselects RED FIELD; `BlueAuto.java` registers BLUE AUTO and preselects BLUE FIELD.

1. Set that alliance's actual initial placement in `ShootingConfig` or Panels before INIT.
2. Paste path poses/methods into the marked section. Use the provided `poseFactory` (inches/degrees), `start`, and the installed Pedro 3 API.
3. Add `follow(follower, path1())` calls to the marked `autoRoutine()` sequence. Empty templates stay stationary. Add mechanism actions when needed.
4. Keep the existing update loop and cleanup: they save actual measured `follower.pose()` every loop and take a final odometry-only sample on exit, including an interrupted path.

`PoseStorage.redPose` goes only to RED FIELD; `bluePose` goes only to BLUE FIELD. X/Y/heading transfer unchanged. Values persist between OpModes in the same Robot Controller app process; an app restart clears them. Set the corresponding variable to `null` to use the manual-placement fallback. The original AutoPath1 remains separate and does not save alliance poses.

## Field coordinates and sources

Frame: origin at the audience/red corner, +X toward the blue wall, +Y away from the audience. Heading 0 faces +X; CCW is positive. See [Pedro coordinates](https://pedropathing.com/docs/pathing/reference/coordinates).

| Alliance/operator station | LEFT | RIGHT |
|---|---|---|
| Red at X=0 | (59.25, 87.77) | (59.25, 56.23) |
| Blue at X=144 | (84.75, 56.23) | (84.75, 87.77) |

These are nominal engineering aim points, not surveyed scoring centers. From a 144-inch field and 25.5-inch hive spacing, X = 72 ± 25.5/2. Opening height midpoint = (53.5+65.6)/2 = 59.55 inches. Using level half-span 42.91/2, pivot height 43.95 and tilt 30°, horizontal offset = `(21.455-(59.55-43.95)*sin(30°))/cos(30°)` ≈ 15.77, giving Y = 72 ± 15.77. Confirm target coordinates on your field.

- [FIRST BIOBUZZ TU06 manual](https://ftc-resources.firstinspires.org/ftc/game/cm-html), sections 9.1–9.6; [Figure 9-9](https://ftc-resources.firstinspires.org/ftc/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU06_files/image029.png), [9-10](https://ftc-resources.firstinspires.org/ftc/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU06_files/image030.png), [9-11](https://ftc-resources.firstinspires.org/ftc/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU06_files/image031.png).
- [FIRST field CAD/resources](https://ftc-resources.firstinspires.org/ftc/archive/2027/field), inspected STEP archive v26-27.2.
- [Event Field Setup Guide](https://ftc-resources.firstinspires.org/ftc/archive/2027/field/eventfieldguide), sections 11.1 and 12.1.

## Verification

`./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug`

On the robot: first run TURRET TUNER to set `turretTicksPerDeg` and `turretReversed`, set safe limits for your wiring, then tune PID starting at low `turretMaxPower`. Then verify each alliance/cell, that the turret stays on target while driving and rotating, the limit warning, auto→TeleOp turret zero handoff, speed recovery, and LT/X priority. Moving-shot accuracy needs physical testing; projectile velocity compensation is not implemented.
