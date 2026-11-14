package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlowerIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Lillian", group = "Autonomous")
public class Lillian extends LinearOpMode {

    private Follower follower;
    private ScoringSubsystem shooter;
    private Intake intake;
    private ScoringIntakeSubsystem scoringIntake;
    private FlowerIntake flower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.2066, 7.7082, 90);
    private final Pose point1 = poseFactory.of(55.2179, 14.3209, 89.9017);
    private final Pose path1Start = poseFactory.of(55.2179, 14.3209, 90);
    private final Pose path1 = poseFactory.of(13.8439, 19.0614, -90);
    private final Pose point3Start = poseFactory.of(13.8439, 19.0614, -90);
    private final Pose point3 = poseFactory.of(9.0438, 10.87, -90);
    private final Pose point4Start = poseFactory.of(9.0438, 10.87, -90);
    private final Pose point4 = poseFactory.of(59.1391, 117.1155, -90);
    private final Pose point4Control1 = poseFactory.of(28.5932, 130.3998, 0);
    private final Pose point5Start = poseFactory.of(59.1391, 117.1155, 90);
    private final Pose point5 = poseFactory.of(47.0925, 132.306, 90);
    private final Pose point5Control1 = poseFactory.of(44.9773, 119.3234, 0);
    private final Pose point6Start = poseFactory.of(47.0925, 132.306, 90);
    private final Pose point6 = poseFactory.of(46.4132, 125.522, 90);
    private final Pose point7Start = poseFactory.of(46.4132, 125.522, -90);
    private final Pose point7 = poseFactory.of(58.8802, 116.464, -90);
    private final Pose point8Start = poseFactory.of(58.8802, 116.464, -90);
    private final Pose point8 = poseFactory.of(10.743, 106.9787, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                flowerInit(),
                shootStart(),
                intakeBalls(),
                waitMs(3000),
                shootBalls(),
                waitMs(3000),
                stopShoot(),
                waitMs(1500),
                follow(follower, path1()),
                follow(follower, path1_2()),
                follow(follower, path3()),
                follow(follower, path4()),
                waitMs(3000),
                shootBalls(),
                waitMs(3000),
                stopShoot(),
                waitMs(1500),
                flowerStart(),
                follow(follower, path5()),
                flowerInit(),
                follow(follower, path6()),
                waitMs(3000),
                shootBalls(),
                waitMs(3000),
                stopShoot(),
                waitMs(1500),
                follow(follower, path7())
        );
    }
    private Command shootStart() {
        return instant(() -> shooter.setShooterState(ScoringSubsystem.ShooterState.FORWARD))
                .requiring(shooter);
    }
    private Command flowerInit() {
        return instant(() -> flower.setState(FlowerIntake.FlowerState.RETRACT))
                .requiring(flower);
    }
    private Command flowerStart() {
        return instant(() -> flower.setState(FlowerIntake.FlowerState.INTAKE))
                .requiring(flower);
    }
    private Command intakeBalls() {
        /*return Command.build()
                .setExecute(() -> {
                    intake.setState(Intake.IntakeState.BACKWARD);
                })
                .setDone(() -> {
                    return true;
                })
                .requiring(intake);*/
        return instant(() -> intake.setState(Intake.IntakeState.SHOOT))
                .requiring(intake);

    }

    private Command shootBalls(){
        /*return Command.build()
                .setExecute(() -> {
                    scoringIntake.setScoringIntakeState(ScoringIntakeSubsystem.ScoringIntakeState.SHOOT);
                })
                .setDone(() -> {
                    return true;
                })
                .requiring(scoringIntake);*/
        return instant(() -> scoringIntake.setScoringIntakeState(ScoringIntakeSubsystem.ScoringIntakeState.SHOOT))
                .requiring(scoringIntake);

    }

    private Command stopShoot(){
        /*return Command.build()
                .setExecute(() -> {
                    scoringIntake.setScoringIntakeState(ScoringIntakeSubsystem.ScoringIntakeState.INIT);
                })
                .setDone(() -> {
                    return true;
                })
                .requiring(scoringIntake);*/
        return instant(() -> scoringIntake.setScoringIntakeState(ScoringIntakeSubsystem.ScoringIntakeState.INIT))
                .requiring(scoringIntake);

    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        intake = new Intake(hardwareMap);
        scoringIntake = new ScoringIntakeSubsystem(hardwareMap);
        shooter = new ScoringSubsystem(hardwareMap);
        flower = new FlowerIntake(hardwareMap);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, point1).tangent();
    }

    public Path path1_2() {
        return Paths.line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point7Start, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point8Start, point8).linear(point8Start, point8);
    }
}