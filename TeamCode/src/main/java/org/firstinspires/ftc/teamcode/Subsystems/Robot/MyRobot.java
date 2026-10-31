package org.firstinspires.ftc.teamcode.Subsystems.Robot;

import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveCommand.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands.AimAndShootCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem.ScoringIntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShootingConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShotTargeting;

public class MyRobot extends Robot {
    private final LinearOpMode opMode;
    private final DriveSubsystem drive;
    private final Intake intake;
    private final ScoringSubsystem shooter;
    private final ScoringIntakeSubsystem feeder;
    private final ShotTargeting targeting = new ShotTargeting();
    private ShotTargeting.Solution shot;
    private final AimAndShootCommand aimAndShoot;
    private String mode = "Manual drive";

    public MyRobot(LinearOpMode opMode, Pose startPose) {
        reset();
        enable();
        this.opMode = opMode;
        drive = new DriveSubsystem(opMode.hardwareMap, startPose);
        intake = new Intake(opMode.hardwareMap);
        shooter = new ScoringSubsystem(opMode.hardwareMap);
        feeder = new ScoringIntakeSubsystem(opMode.hardwareMap);
        aimAndShoot = new AimAndShootCommand(drive, feeder, () -> shot, shooter::getVelocity);

        drive.setDefaultCommand(new DefaultDriveCommand(drive,
                () -> opMode.gamepad1.left_stick_x, () -> -opMode.gamepad1.left_stick_y,
                () -> opMode.gamepad1.right_stick_x, () -> opMode.gamepad1.left_bumper));
        shooter.setDefaultCommand(new SpinShooterCommand(shooter, () -> shot.velocityTicksPerSecond));
        intake.setDefaultCommand(new RunCommand(() -> intake.setState(Intake.IntakeState.BACKWARD), intake));
        feeder.setDefaultCommand(new RunCommand(() -> feeder.setScoringIntakeState(ScoringIntakeState.INIT), feeder));

        // Priority: reverse > manual shot > automatic shot. Each owns only the hardware it uses.
        new Trigger(() -> opMode.gamepad1.x).whileActiveContinuous(new FunctionalCommand(
                () -> {},
                () -> {
                    intake.setState(Intake.IntakeState.FORWARD);
                    feeder.setScoringIntakeState(ScoringIntakeState.REVERSE);
                },
                interrupted -> {
                    intake.setState(Intake.IntakeState.BACKWARD);
                    feeder.setScoringIntakeState(ScoringIntakeState.INIT);
                },
                () -> false, intake, feeder));
        new Trigger(() -> !opMode.gamepad1.x && manualShotRequested())
                .whileActiveContinuous(new FunctionalCommand(
                        () -> {},
                        () -> feeder.setScoringIntakeState(ScoringIntakeState.SHOOT),
                        interrupted -> feeder.setScoringIntakeState(ScoringIntakeState.INIT),
                        () -> false, feeder));
        new Trigger(() -> !opMode.gamepad1.x && !manualShotRequested() && autoAimRequested())
                .whileActiveContinuous(aimAndShoot);
    }

    public void update(double targetX, double targetY, String shots, boolean calibrated) {
        drive.updatePose();
        shot = targeting.calculate(drive.getPose(), targetX, targetY, shots, calibrated);
        // FTCLib applies live PIDF in periodic(), polls buttons, and runs the commands.
        run();
        mode = opMode.gamepad1.x ? "Reverse" : manualShotRequested() ? "Manual shot"
                : autoAimRequested() ? "Auto aim" : "Manual drive";
    }

    public void resetPose(Pose pose) {
        drive.setPose(pose);
        drive.beginAim();
    }

    private boolean manualShotRequested() {
        return opMode.gamepad2.left_trigger > ShootingConfig.triggerThreshold;
    }

    private boolean autoAimRequested() {
        return opMode.gamepad2.right_trigger > ShootingConfig.triggerThreshold;
    }

    public void showTelemetry(String alliance, boolean left) {
        Pose pose = drive.getPose();
        double actual = shooter.getVelocity();
        opMode.telemetry.addData("TeleOp / cell", "%s / %s", alliance, left ? "LEFT" : "RIGHT");
        opMode.telemetry.addData("Pedro pose (in, deg)", "%.2f, %.2f, %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        opMode.telemetry.addData("Distance (in)", "%.2f", shot.distanceIn);
        opMode.telemetry.addData("Aim heading / error (deg)", "%.2f / %.2f",
                Math.toDegrees(shot.headingRadians), Math.toDegrees(shot.errorRadians));
        opMode.telemetry.addData("Shooter target / actual (ticks/s)", "%.1f / %.1f",
                shooter.getTargetVelocity(), actual);
        opMode.telemetry.addData("Mode / feeder", "%s / %s", mode, feeder.getState());
        opMode.telemetry.addData("Pose reset", "Driver Y: corner pose");
        opMode.telemetry.addData("Auto ready", shot.ready(actual));
        opMode.telemetry.addData("Warning", shot.warning == null ? "None" : shot.warning);
        opMode.telemetry.addData("PIDF", shooter.isPidfValid() ? "Applied" : "Invalid edit; keeping last gains");
        opMode.telemetry.update();
    }

    public void stopAll() {
        reset();
        drive.stop();
        shooter.setVelocity(0);
        feeder.setScoringIntakeState(ScoringIntakeState.INIT);
        intake.setState(Intake.IntakeState.INIT);
    }
}
