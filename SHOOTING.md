# Shooting and autonomous setup

Distances use Pedro field inches (X/Y 0–144). Panels angles are degrees; Pedro Pose headings are radians.
All live settings are in `Subsystems/Scoring/ShootingConfig.java`.

## Controls

- Goal selection is automatic: each TeleOp aims at whichever of its own alliance's two goals is nearer the turret pivot (RED FIELD only red goals, BLUE FIELD only blue). It switches only when the other goal is `goalSwitchMarginIn` closer, so it does not flip near the midline. Telemetry "TeleOp / auto goal" shows the pick. LEFT/RIGHT are from the alliance station looking onto the field. Operator bumpers do nothing.
- Turret: the motor turret always points at the selected target from odometry. Every loop it recalculates the robot-relative target angle from the current pose, then sets power = PID(target − measured angle). No button, and the drivetrain is never turned to aim.
- Wrap-around: of every equivalent target (angle ± 360°) inside `turretMinDeg..turretMaxDeg`, the turret goes to the one nearest its current angle, never past a limit. If none fits, it stops at the closest limit and telemetry says to turn the robot.
- Operator LT: any press feeds immediately; no readiness checks. Telemetry "Turret on target" shows whether the turret is within `turretToleranceDeg` of its command.
- Turret start: turret angle = `turretStartDeg` + encoder ticks / `turretTicksPerDeg` (both read once at INIT). Put the turret at `turretStartDeg` before INIT of any auto; autos always zero it. After that, no OpMode zeros it again in this app session (`PoseStorage.turretZeroed`), whichever alliance runs next. If the first OpMode after an app start is a TeleOp, put the turret at `turretStartDeg` before its INIT.
- Driver X: reverse intake/feeder and keep manual driving. X wins over LT.
- Driver LB: half-speed manual driving.
- Driver Y: set the current X/Y/heading to that alliance's configured corner pose, once per press. Disabled (telemetry says so) while that alliance's reset X and Y are both 0, the unmeasured placeholder.
- Shooter speed follows distance continuously, including while driving, aiming, reversing or being pushed. Front intake runs continuously except when reversed.
- Release LT: feeder stops.
- Field-centric drive from the odometry heading, using Pedro's `follower.manual(ManualDrive.fieldCentric(..., heading, offset))` then `follower.update()`. Offset is 0 for red and 180° for blue, so stick-forward always drives away from your own driver station. No auto-hold on stick release. Nothing moves during INIT.
- Start pose: TeleOp starts from that alliance's last saved pose (the auto's end, or the previous TeleOp run), else the alliance start pose. The INIT screen shows which.

## Configuration

| Setting | Meaning |
|---|---|
| `redResetXIn/YIn/HeadingDeg`, `blueResetXIn/YIn/HeadingDeg` | Driver Y corner-reset pose, independent per alliance and editable live in Panels. Zero X and Y = not measured, Y disabled. |
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
| `goalSwitchMarginIn` | Auto goal switch margin; default 6 in. 0 = always the nearest goal; larger = less flipping but holds the old goal longer. |
| Four `...Shots` strings | Measured distance:velocity pairs, e.g. `10:500,50:900,100:1200`, inches:ticks/second. Distances increase; speeds are positive. |
| Four `...Calibrated` flags | Telemetry reminders only; false does not block feeding. |
| `kp/ki/kd/kf` | Live shooter PIDF; changed valid values are sent to the motor controller. |
| `autoSpinUpMs`, `autoSettleMs`, `autoFeedMs` | Auto shots: wait before the first shot (default 1500), wait before the later shots (500), feeder run time per shot (1500). |

Shooter offsets and shot tables are placeholders. The shipped `70:950,76.05:1000` tables reproduce two points on the old `origin/Shooting_Formula` equation; they are not measured shots. Measure shots at several distances and approach angles for each cell, then update the tables. Record physical release height and launch angle in your calibration notes; changing them requires recalibration. They are not live controls because this code uses measured speed tables.

Outside a table's range, speed clamps to its nearest endpoint and telemetry warns. Invalid tables or non-finite geometry command zero speed. There is no speed cap: keep table speeds within what the flywheel can reach. LT always feeds, using the same distance-derived speed; it cannot reconstruct distance from broken odometry. On invalid geometry the turret brakes in place. Invalid PIDF edits retain the last valid gains while speed updates continue.

## Code flow

- `Alliance.java`: everything red/blue: start pose, goals, Y reset pose, saved pose, drive offset, and `fromRed()` (blue = red rotated 180° about (72, 72)).
- `AllianceTeleOp.java`: shared TeleOp; `RedTeleop` / `BlueTeleop` only pick the alliance.
- `GoalTracker.java` + `ShotTargeting.pickLeft()`: nearest-goal choice with the switch margin, used by TeleOp and auto.
- `MyRobot.update()`: update odometry and save the pose, pick the nearer goal, calculate the shot, apply live PIDF and speed, aim the turret, then handle X → LT.
- `ShotTargeting.java`: plain geometry/interpolation helper. Warning text does not control feeding.
- `DriveSubsystem.java`: Pedro field-centric manual drive with the alliance offset.
- `TurretSubsystem.java` / `AimTurretCommand.java`: pick the wrap-around target (`ShotTargeting.nearestTurretAngle`) and run one PID step on `TurretMotor` each loop. D acts on the measured angle (no kick when the target jumps). Past a travel limit, only power back toward the range is allowed. Invalid limits (min > max, or more than 720° apart) brake the turret. The same odometry sample determines turret angle and speed.
- `TurretTuner.java` (TURRET TUNER): during INIT the motor floats; turn the turret exactly 90° CCW by hand and read ticks/deg. After START, dpad left/up/right/down = 90/0/−90/180° to tune PID live.
- `ScoringSubsystem.java`: motor velocity and live PIDF. Speed is set directly each loop. Autos use the same distance tables as TeleOp.
- STOP uses the normal FTC loop exit and directly stops drive, shooter and intake outputs.

## Autonomous and pose handoff

`AllianceAuto.java` holds the one auto path, written in RED coordinates from the Pedro visualizer. `RedAuto` (RED AUTO → RED FIELD) runs it as written; `BlueAuto` (BLUE AUTO → BLUE FIELD) runs every pose through `Alliance.fromRed()`, i.e. (144 − x, 144 − y, heading + 180°). The BIOBUZZ field is 180° rotationally symmetric (manual Figure 9-2: red loading zone on the back of the left wall, blue on the front of the right wall; gardens in opposite corners), so this keeps blue odometry correct.

- Placement: red (55.2, 7.7, 90°) against the audience wall; blue (88.8, 136.3, −90°) against the back wall. The INIT screen shows the pose to place the robot at.
- Routine: shoot, path1, path1_2, path3, path4, shoot, path5, path6, path7, shoot, path8. Turret and flywheel track the nearest goal the whole time; intake runs the whole time. Each shot waits `autoSpinUpMs` (first) or `autoSettleMs`, then feeds for `autoFeedMs`.
- path5 and path7 start with a heading 180° from where the previous path ends, so the robot spins in place there. Change the path if that is not intended.
- To edit the path, change the red coordinates in `AllianceAuto`; blue follows automatically.

`PoseStorage.redPose` goes only to RED FIELD; `bluePose` only to BLUE FIELD. Autos and TeleOps save the measured pose every loop and on exit, so a later TeleOp run resumes where the robot really is. Values persist in the same Robot Controller app process; an app restart clears them.

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

On the robot: check stick-forward drives away from the driver on both alliances. Run RED AUTO and BLUE AUTO and confirm the pose on each INIT screen matches the placement. First run TURRET TUNER to set `turretTicksPerDeg` and `turretReversed`, set safe limits for your wiring, then tune PID starting at low `turretMaxPower`. Then verify each alliance/cell, that the auto goal switches once when driving between goals and does not flicker on the midline, that the turret stays on target while driving and rotating, the limit warning, auto→TeleOp turret zero handoff, speed recovery, and LT/X priority. Moving-shot accuracy needs physical testing; projectile velocity compensation is not implemented.
