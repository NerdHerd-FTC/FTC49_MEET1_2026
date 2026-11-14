package org.firstinspires.ftc.teamcode.Subsystems.Robot;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShootingConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShotTargeting;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

/**
 * Everything that differs between red and blue. Pedro field: origin at the audience/red corner,
 * +X toward the blue wall, +Y away from the audience. The BIOBUZZ field is 180-degree
 * rotationally symmetric (manual Figure 9-2), so blue = red rotated about the center (72, 72).
 */
public enum Alliance {
    RED, BLUE;

    /** Red path start, from the Pedro visualizer path. Blue's start is this rotated. */
    public static final Pose RED_START = new Pose(55.2066, 7.7082, Math.toRadians(90));

    /** Converts a pose written in red coordinates to this alliance. */
    public Pose fromRed(Pose red) {
        if (this == RED) return red;
        return new Pose(144 - red.x(), 144 - red.y(), ShotTargeting.wrap(red.heading() + Math.PI));
    }

    public Pose startPose() { return fromRed(RED_START); }

    /** Field-centric offset so stick-forward points away from this alliance's driver station. */
    public double driveOffsetRadians() { return this == RED ? 0 : Math.PI; }

    // LEFT/RIGHT are from this alliance's station looking onto the field.
    public ShotTargeting.Goal leftGoal() {
        return this == RED
                ? new ShotTargeting.Goal(ShootingConfig.redLeftXIn, ShootingConfig.redLeftYIn,
                        ShootingConfig.redLeftShots, ShootingConfig.redLeftCalibrated)
                : new ShotTargeting.Goal(ShootingConfig.blueLeftXIn, ShootingConfig.blueLeftYIn,
                        ShootingConfig.blueLeftShots, ShootingConfig.blueLeftCalibrated);
    }

    public ShotTargeting.Goal rightGoal() {
        return this == RED
                ? new ShotTargeting.Goal(ShootingConfig.redRightXIn, ShootingConfig.redRightYIn,
                        ShootingConfig.redRightShots, ShootingConfig.redRightCalibrated)
                : new ShotTargeting.Goal(ShootingConfig.blueRightXIn, ShootingConfig.blueRightYIn,
                        ShootingConfig.blueRightShots, ShootingConfig.blueRightCalibrated);
    }

    /** Driver Y corner pose, or null while it is still the (0, 0) placeholder. */
    public Pose resetPose() {
        double x = this == RED ? ShootingConfig.redResetXIn : ShootingConfig.blueResetXIn;
        double y = this == RED ? ShootingConfig.redResetYIn : ShootingConfig.blueResetYIn;
        double h = this == RED ? ShootingConfig.redResetHeadingDeg : ShootingConfig.blueResetHeadingDeg;
        // A robot's center can never be at the field corner, so (0, 0) means "not measured yet".
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(h) || (x == 0 && y == 0)) {
            return null;
        }
        return new Pose(x, y, Math.toRadians(h));
    }

    /** Last pose saved by an auto or TeleOp of this alliance, or null if none this app session. */
    public Pose savedPose() { return this == RED ? PoseStorage.redPose : PoseStorage.bluePose; }

    public void savePose(Pose pose) {
        if (this == RED) PoseStorage.redPose = pose;
        else PoseStorage.bluePose = pose;
    }
}
