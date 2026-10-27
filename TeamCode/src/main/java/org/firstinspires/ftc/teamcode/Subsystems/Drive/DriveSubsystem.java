package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import static com.pedropathing.api.Paths.line;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveSubsystem extends SubsystemBase {
    private MyRobot robot;
    private Follower follower;
    public boolean fieldOriented = true;
    public boolean autoDriving;
    public ElapsedTime autoTimer = new ElapsedTime();
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(133, 108, 0);
    private final Pose scorePose = poseFactory.of(83,130,270);

    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore())
                // Add mechanism commands here
        );
    }
    public DriveSubsystem(MyRobot robot){
        follower = Constants.create(robot.hardwareMap);
        follower.setPose(startPose);
    }

    public void setDrivePower(double leftX, double leftY, double rightX){
        if (fieldOriented) {
            DrivePowers powers = ManualDrive.fieldCentric(
                    leftY,
                    -leftX,
                    -rightX,
                    follower.pose().heading()
            );

            ManualDrive.driveOrHold(follower, powers);
            follower.update();
        } else {
            ManualDrive.driveOrHold(follower,
                    leftY,
                    -leftX,
                    -rightX
            );
            follower.update();
        }

    }

    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }
    public boolean isReady() { return follower != null; }

    public void driveTo() {
        Scheduler.reset();
        schedule(autoRoutine());
        follower.update();
        Scheduler.execute();
    }

}
