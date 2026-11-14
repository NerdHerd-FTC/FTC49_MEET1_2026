package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSubsystem extends SubsystemBase {
    private final Follower follower;
    private final double fieldOffsetRadians;

    /**
     * @param fieldOffsetRadians field-centric offset so stick-forward points away from this
     *                           alliance's driver station (0 red, PI blue).
     */
    public DriveSubsystem(HardwareMap hardwareMap, Pose startPose, double fieldOffsetRadians) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        this.fieldOffsetRadians = fieldOffsetRadians;
    }

    /**
     * Called once at the start of the TeleOp loop: updates odometry and applies the last manual
     * powers. Pedro's documented TeleOp flow is follower.manual(...) then follower.update().
     * Until the first manual command the follower is idle, so nothing moves.
     */
    public void updatePose() {
        follower.update();
    }

    public Pose getPose() { return follower.pose(); }
    public void setPose(Pose pose) { follower.setPose(pose); }

    public void setDrivePower(double leftX, double leftY, double rightX) {
        // Manual mode only; no auto-hold, so the robot never fights the driver on stick release.
        follower.manual(ManualDrive.fieldCentric(leftY, -leftX, -rightX,
                getPose().heading(), fieldOffsetRadians));
    }

    public void stop() {
        follower.stop();
        follower.drivetrain.stop();
    }
}
