package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShootingConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final Follower follower;
    private final Foresight holdController;
    private long lastUpdate;
    private double deltaTime;

    public DriveSubsystem(HardwareMap hardwareMap, Pose startPose) {
        holdController = new Foresight(Constants.foresightConfig);
        follower = Constants.create(hardwareMap, holdController);
        follower.setPose(startPose);
    }

    /** Called once at the start of the TeleOp loop. */
    public void updatePose() {
        long now = System.nanoTime();
        deltaTime = lastUpdate == 0 ? 0 : (now - lastUpdate) / 1e9;
        lastUpdate = now;
        follower.localizer.update();
    }

    public Pose getPose() { return follower.pose(); }
    public void setPose(Pose pose) { follower.setPose(pose); }

    public void setDrivePower(double leftX, double leftY, double rightX) {
        DrivePowers powers = ManualDrive.fieldCentric(leftY, -leftX, -rightX, getPose().heading());
        // Explicit manual output avoids re-entering position hold on trigger release.
        follower.drivetrain.drive(powers, true);
    }

    public void beginAim() { holdController.reset(); }

    public void driveToHeading(double heading) {
        double scale = ShootingConfig.aimHeadingScale;
        if (!Double.isFinite(heading) || !Double.isFinite(scale) || scale < 0 || scale > 1) {
            stop();
            return;
        }
        double translationScale = Constants.foresightConfig.holdPointTranslationalScaling.get();
        double headingScale = Constants.foresightConfig.holdPointHeadingScaling.get();
        try {
            Constants.foresightConfig.holdPointTranslationalScaling.set(0.0);
            Constants.foresightConfig.holdPointHeadingScaling.set(scale);
            Pose pose = getPose();
            // Same Pedro controller as Follower.hold(). Direct evaluation avoids a second
            // localizer read: aiming and feeding use the SAME pose sample in this loop.
            DrivePowers powers = holdController.calculateHold(follower.drivetrain,
                    new Pose(pose.x(), pose.y(), heading), follower.localizer.state(), true, deltaTime);
            follower.drivetrain.drive(powers, false);
        } finally {
            Constants.foresightConfig.holdPointTranslationalScaling.set(translationScale);
            Constants.foresightConfig.holdPointHeadingScaling.set(headingScale);
        }
    }

    public void stop() { follower.drivetrain.stop(); }
}
