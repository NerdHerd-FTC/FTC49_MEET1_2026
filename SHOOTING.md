# Shooting and autonomous setup

Distances use Pedro field inches (X/Y 0–144). Panels angles are degrees; Pedro Pose headings are radians.
All live settings are in `Subsystems/Scoring/ShootingConfig.java`.

## Controls

- Operator LB selects LEFT, RB selects RIGHT, from their own alliance station looking onto the field. Selection persists; default is RIGHT.
- Operator RT: align and feed immediately when heading error is within ±2.5° inclusive and actual shooter speed is within tolerance. No timer or stillness requirement.
- Operator LT alone: feed while keeping manual driving; bypass automatic readiness. LT wins over RT.
- Driver X: reverse intake/feeder and keep manual driving. X wins over both triggers.
- Driver LB: half-speed manual driving.
- Driver Y: set the current X/Y/heading to that alliance's configured corner pose, once per press. Place the robot at the measured corner position and facing before pressing.
- Shooter speed follows distance continuously, including while driving, aiming, reversing or being pushed. Front intake runs continuously except when reversed.
- Release shooting triggers: manual driving and stopped feeder.
- Both TeleOps use the same Pedro field-centric axes and current global heading, with no alliance joystick offset.

## Configuration

| Setting | Meaning |
|---|---|
| `redStartXIn/YIn/HeadingDeg`, `blueStartXIn/YIn/HeadingDeg` | Actual auto starting placement; also TeleOp fallback if no auto pose was saved. Set before INIT. |
| `redResetXIn/YIn/HeadingDeg`, `blueResetXIn/YIn/HeadingDeg` | Driver Y corner-reset pose, independent per alliance and editable live in Panels. All six defaults are zero placeholders; measure Pedro's robot reference point at the corner and its heading before using. |
| `shooterForwardIn`, `shooterLeftIn` | Shooter release point relative to Pedro's robot reference point. Forward/left are positive. |
| `shooterYawOffsetDeg` | Shooter direction relative to robot forward; CCW positive. |
| Four `red/blueLeft/RightXIn/YIn` pairs | Aim-point coordinates for each alliance/cell. |
| Four `...Shots` strings | Measured distance:velocity pairs, e.g. `10:500,50:900,100:1200`, inches:ticks/second. Distances increase; speeds are positive. |
| Four `...Calibrated` flags | Telemetry reminders only; false does not block feeding. |
| `headingToleranceDeg`, `velocityToleranceTicksPerSecond` | Automatic feeding windows; defaults 2.5° and 50 ticks/second. |
| `triggerThreshold` | Trigger activation level; default 0.2. |
| `aimHeadingScale` | Pedro heading-hold output scaling, 0–1; default 0.35. |
| `maxVelocityTicksPerSecond` | Reject a calculated target above this limit; provisional default 2000. |
| `autonomousVelocityTicksPerSecond` | Fixed speed used by the existing legacy AutoPath1 mechanism commands; default 950. |
| `kp/ki/kd/kf` | Live shooter PIDF; changed valid values are sent to the motor controller. |

Starting poses, shooter offsets and shot tables are placeholders. The shipped `70:950,76.05:1000` tables reproduce two points on the old `origin/Shooting_Formula` equation; they are not measured shots. Measure shots at several distances and approach angles for each cell, then update the tables. Record physical release height and launch angle in your calibration notes; changing them requires recalibration. They are not live controls because this code uses measured speed tables.

Outside a table's range, speed clamps to its nearest endpoint and telemetry warns; automatic feeding is still allowed when heading/speed are ready. Invalid tables, non-finite geometry or an invalid/exceeded speed limit command zero speed and prevent automatic feeding. LT bypasses readiness but still uses the same distance-derived speed; it cannot reconstruct distance from broken odometry. Invalid PIDF edits retain the last valid gains while speed updates continue.

## Code flow

- `RedTeleop.java` / `BlueTeleop.java`: initialize from their own saved pose, read bumper selection and select their own target/table.
- `MyRobot.update()`: update odometry, calculate the shot, apply live PIDF and speed, then handle X → LT → RT → manual driving in that order.
- `ShotTargeting.java`: plain geometry/interpolation helper. Its result has a boolean `ready()`; warning text does not control feeding.
- `DriveSubsystem.java`: Pedro field-centric manual drive and heading-only aim. Aiming uses Pedro Foresight hold at current X/Y with zero translational scaling, so it does not pull back against pushing. The same odometry sample determines aim, speed and feed readiness.
- `ScoringSubsystem.java`: motor velocity and live PIDF. Speed is set directly each loop.
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

On the robot: verify each alliance/cell and turn direction, push sideways during RT aiming, test ±2.5° boundaries and speed recovery, and check LT/X priority and trigger release. There is no dwell or hysteresis, so watch for feeder chatter at the tolerance boundary. Moving-shot accuracy needs physical testing; projectile velocity compensation is not implemented.
