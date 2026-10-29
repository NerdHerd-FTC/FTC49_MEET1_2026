package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShootingConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

@Autonomous(name = "BLUE AUTO", group = "AA_Drive_Code", preselectTeleOp = "BLUE FIELD")
public class BlueAuto extends LinearOpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Pose start;

    public Command autoRoutine() {
        return sequential(
                // PASTE FOLLOW COMMANDS HERE, in order:
                // follow(follower, path1()),
                // follow(follower, path2())
        );
    }

    // PASTE PATH POSES AND METHODS HERE. Coordinates are Pedro field inches.
    // Example (replace the end pose with your real path):
    // private final Pose end = poseFactory.of(50, 50, 90);
    // public Path path1() { return Paths.line(start, end).linear(start, end); }

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        try {
            // Set this alliance's Start values in Panels BEFORE INIT to match placement.
            start = poseFactory.of(ShootingConfig.blueStartXIn, ShootingConfig.blueStartYIn,
                    ShootingConfig.blueStartHeadingDeg);
            follower.setPose(start);
            follower.localizer.update();
            PoseStorage.bluePose = follower.pose();
            showPose();

            waitForStart();
            if (isStopRequested()) return;
            Scheduler.schedule(autoRoutine());

            // Keep updating even after the paths finish, until FTC STOP.
            while (opModeIsActive()) {
                Scheduler.execute();
                follower.update();
                PoseStorage.bluePose = follower.pose();
                showPose();
            }
        } finally {
            follower.drivetrain.stop();
            Scheduler.reset();
            // Sample odometry without issuing another drive command after STOP.
            follower.localizer.update();
            PoseStorage.bluePose = follower.pose();
        }
    }

    private void showPose() {
        Pose pose = follower.pose();
        telemetry.addData("Auto", "BLUE AUTO -> BLUE FIELD");
        telemetry.addData("Saved pose (in, deg)", "%.2f, %.2f, %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.update();
    }
}
