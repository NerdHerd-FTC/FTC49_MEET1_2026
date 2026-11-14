package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.GoalTracker;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem.ScoringIntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShootingConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShotTargeting;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Shared auto; RedAuto and BlueAuto only choose the alliance. The path is written in RED
 * coordinates (from the Pedro visualizer) and every pose goes through at(), which rotates it
 * 180 degrees about the field center for blue. The turret and flywheel track the nearest
 * alliance goal the whole time; the feeder runs at the start and after path4 and path7.
 */
public abstract class AllianceAuto extends LinearOpMode {
    protected abstract Alliance alliance();

    private Follower follower;
    private ScoringIntakeSubsystem feeder;

    /** A red-coordinate pose (inches, degrees), converted to this alliance. */
    private Pose at(double x, double y, double headingDeg) {
        return alliance().fromRed(new Pose(x, y, Math.toRadians(headingDeg)));
    }

    public Command autoRoutine() {
        return sequential(
                shoot(ShootingConfig.autoSpinUpMs),
                follow(follower, path1()),
                follow(follower, path1_2()),
                follow(follower, path3()),
                follow(follower, path4()),
                shoot(ShootingConfig.autoSettleMs),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7()),
                shoot(ShootingConfig.autoSettleMs),
                follow(follower, path8())
        );
    }

    /** Wait for the turret/flywheel to settle, then run the feeder. */
    private Command shoot(double settleMs) {
        return sequential(
                waitMs(settleMs),
                instant(() -> feeder.setScoringIntakeState(ScoringIntakeState.SHOOT)),
                waitMs(ShootingConfig.autoFeedMs),
                instant(() -> feeder.setScoringIntakeState(ScoringIntakeState.INIT))
        );
    }

    // Red-coordinate path from the Pedro visualizer. Note path5 and path7 start with a heading
    // 180 degrees from where the previous path ends, so the robot spins in place there.
    public Path path1() {
        // Starts from Alliance.RED_START (the one place the start pose is written).
        return Paths.line(alliance().startPose(), at(55.2179, 14.3209, 89.9017)).tangent();
    }

    public Path path1_2() {
        Pose start = at(55.2179, 14.3209, 90), end = at(13.8439, 19.0614, -90);
        return Paths.line(start, end).linear(start, end);
    }

    public Path path3() {
        Pose start = at(13.8439, 19.0614, -90), end = at(9.0438, 10.87, -90);
        return Paths.line(start, end).linear(start, end);
    }

    public Path path4() {
        Pose start = at(9.0438, 10.87, -90), end = at(59.1391, 117.1155, -90);
        return Paths.curve(start, at(28.5932, 130.3998, 0), end).linear(start, end);
    }

    public Path path5() {
        Pose start = at(59.1391, 117.1155, 90), end = at(47.0925, 132.306, 90);
        return Paths.curve(start, at(44.9773, 119.3234, 0), end).linear(start, end);
    }

    public Path path6() {
        Pose start = at(47.0925, 132.306, 90), end = at(46.4132, 125.522, 90);
        return Paths.line(start, end).linear(start, end);
    }

    public Path path7() {
        Pose start = at(46.4132, 125.522, -90), end = at(58.8802, 116.464, -90);
        return Paths.line(start, end).linear(start, end);
    }

    public Path path8() {
        Pose start = at(58.8802, 116.464, -90), end = at(10.743, 106.9787, 0);
        return Paths.line(start, end).linear(start, end);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Alliance alliance = alliance();
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        // Put the turret at turretStartDeg before INIT: auto always zeros its encoder for itself and TeleOp.
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, true);
        ScoringSubsystem shooter = new ScoringSubsystem(hardwareMap);
        feeder = new ScoringIntakeSubsystem(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        GoalTracker goals = new GoalTracker();
        try {
            // Follower is idle until a path starts, so this update moves nothing.
            follower.setPose(alliance.startPose());
            follower.update();
            alliance.savePose(follower.pose());
            while (opModeInInit()) {
                Pose pose = follower.pose();
                telemetry.addData("Auto", "%s AUTO -> %s FIELD", alliance, alliance);
                telemetry.addData("Place robot at (in, deg)", "%.1f, %.1f, %.0f",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addData("Turret", "must sit at turretStartDeg = %.0f", ShootingConfig.turretStartDeg);
                telemetry.update();
            }
            if (isStopRequested()) return;
            Scheduler.schedule(autoRoutine());

            // Keep updating even after the paths finish, until FTC STOP.
            while (opModeIsActive()) {
                follower.update();
                Pose pose = follower.pose();
                alliance.savePose(pose);
                ShotTargeting.Solution shot = goals.update(pose, alliance.leftGoal(), alliance.rightGoal());
                turret.setAngle(shot.turretRadians);
                shooter.periodic();
                shooter.setVelocity(shot.velocityTicksPerSecond);
                intake.setState(Intake.IntakeState.BACKWARD);
                Scheduler.execute();

                telemetry.addData("Pose (in, deg)", "%.1f, %.1f, %.1f",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addData("Goal / distance (in)", "%s / %.1f",
                        goals.isLeft() ? "LEFT" : "RIGHT", shot.distanceIn);
                telemetry.addData("Shooter target / actual", "%.0f / %.0f",
                        shooter.getTargetVelocity(), shooter.getVelocity());
                telemetry.addData("Turret on target", turret.onTarget() && shot.turretReachable);
                if (follower.currentPath() != null) {
                    telemetry.addData("Path / remaining (in)", "%d / %.1f",
                            follower.pathIndex(), follower.distanceToEndpoint());
                }
                telemetry.update();
            }
        } finally {
            follower.drivetrain.stop();
            turret.stop();
            shooter.setVelocity(0);
            feeder.setScoringIntakeState(ScoringIntakeState.INIT);
            intake.setState(Intake.IntakeState.INIT);
            Scheduler.reset();
            // Sample odometry without issuing another drive command after STOP.
            follower.localizer.update();
            alliance.savePose(follower.pose());
        }
    }
}
