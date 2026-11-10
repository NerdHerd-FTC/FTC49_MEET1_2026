package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final Follower follower;

    public DriveSubsystem(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    /** Called once at the start of the TeleOp loop. */
    public void updatePose() {
        follower.localizer.update();
    }

    public Pose getPose() { return follower.pose(); }
    public void setPose(Pose pose) { follower.setPose(pose); }

    public void setDrivePower(double leftX, double leftY, double rightX) {
        DrivePowers powers = ManualDrive.fieldCentric(leftY, -leftX, -rightX, getPose().heading());
        // Explicit manual output avoids re-entering position hold on trigger release.
        follower.drivetrain.drive(powers, true);
    }

    public void stop() { follower.drivetrain.stop(); }
}
