package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.bylazar.configurables.annotations.Configurable;

/** Shooting settings editable in Panels. Distances are inches; angles here are degrees. */
@Configurable
public final class ShootingConfig {
    private ShootingConfig() {}

    // Auto starting placement; also TeleOp fallback when no auto pose is saved. Set BEFORE INIT.
    // These copied placeholders must match actual placement.
    public static volatile double redStartXIn = 133;
    public static volatile double redStartYIn = 108;
    public static volatile double redStartHeadingDeg = 0;
    public static volatile double blueStartXIn = 133;
    public static volatile double blueStartYIn = 108;
    public static volatile double blueStartHeadingDeg = 0;

    // Driver Y corner reset: PLACEHOLDERS. Measure the robot reference point and facing.
    public static volatile double redResetXIn = 0;
    public static volatile double redResetYIn = 0;
    public static volatile double redResetHeadingDeg = 0;
    public static volatile double blueResetXIn = 0;
    public static volatile double blueResetYIn = 0;
    public static volatile double blueResetHeadingDeg = 0;

    // Turret pivot relative to Pedro's robot reference point. Zero offsets are placeholders.
    public static volatile double shooterForwardIn = 0;
    public static volatile double shooterLeftIn = 0;
    // Shooter direction at turret angle 0 relative to robot forward; CCW positive.
    public static volatile double shooterYawOffsetDeg = 0;

    // Turret motor ("TurretMotor"). PLACEHOLDERS: measure on the robot.
    // turretStartDeg: where the turret physically sits at INIT (0 = forward, 90 = left, CCW+).
    // turretTicksPerDeg: measure with TURRET TUNER, or motor ticks/rev x gear ratio / 360.
    // Flip turretReversed if positive power turns the turret clockwise.
    // Limits may span up to 720 deg; wider than 360 lets the turret wrap around.
    public static volatile double turretStartDeg = 0;
    public static volatile double turretTicksPerDeg = 1.494;
    public static volatile boolean turretReversed = false;
    public static volatile double turretMinDeg = -180;
    public static volatile double turretMaxDeg = 180;
    // PID on angle error in degrees -> motor power.
    public static volatile double turretKp = 0.02;
    public static volatile double turretKi = 0;
    public static volatile double turretKd = 0.001;
    public static volatile double turretMaxPower = 0.6;
    public static volatile double turretToleranceDeg = 2;

    // Nominal upward opening bounding-box centers, derived in SHOOTING.md.
    // These are calibration starting points, not surveyed practice-field coordinates.
    // LEFT/RIGHT are from each alliance's driver/operator station looking onto the field.
    public static volatile double redLeftXIn = 59.25;
    public static volatile double redLeftYIn = 87.77;
    public static volatile double redRightXIn = 59.25;
    public static volatile double redRightYIn = 56.23;
    public static volatile double blueLeftXIn = 84.75;
    public static volatile double blueLeftYIn = 56.23;
    public static volatile double blueRightXIn = 84.75;
    public static volatile double blueRightYIn = 87.77;

    // Editable distance:velocity pairs, in inches:ticks/second, strictly increasing in distance.
    // Seeds reproduce the old branch's formula at 70 and 76.05 inches; NOT measured calibration.
    public static volatile String redLeftShots = "70:950,76.05:1000";
    public static volatile String redRightShots = "70:950,76.05:1000";
    public static volatile String blueLeftShots = "70:950,76.05:1000";
    public static volatile String blueRightShots = "70:950,76.05:1000";
    // Calibration reminder in telemetry only; these flags do not prevent feeding.
    public static volatile boolean redLeftCalibrated = false;
    public static volatile boolean redRightCalibrated = false;
    public static volatile boolean blueLeftCalibrated = false;
    public static volatile boolean blueRightCalibrated = false;

    public static volatile double kp = 150;
    public static volatile double ki = 0;
    public static volatile double kd = 0;
    public static volatile double kf = 0;
}
