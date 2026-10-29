package org.firstinspires.ftc.teamcode.Subsystems.Robot;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem.ScoringIntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShootingConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShotTargeting;

public class MyRobot {
    private final LinearOpMode opMode;
    private final DriveSubsystem drive;
    private final Intake intake;
    private final ScoringSubsystem shooter;
    private final ScoringIntakeSubsystem feeder;
    private final ShotTargeting targeting = new ShotTargeting();
    private ShotTargeting.Solution shot;
    private boolean aiming;
    private ScoringIntakeState feederState = ScoringIntakeState.INIT;
    private String mode = "Manual drive";

    public MyRobot(LinearOpMode opMode, Pose startPose) {
        this.opMode = opMode;
        drive = new DriveSubsystem(opMode.hardwareMap, startPose);
        intake = new Intake(opMode.hardwareMap);
        shooter = new ScoringSubsystem(opMode.hardwareMap);
        feeder = new ScoringIntakeSubsystem(opMode.hardwareMap);
    }

    public void update(double targetX, double targetY, String shots, boolean calibrated) {
        drive.updatePose();
        shot = targeting.calculate(drive.getPose(), targetX, targetY, shots, calibrated);
        shooter.periodic(); // Apply live Panels PIDF edits.
        shooter.setVelocity(shot.velocityTicksPerSecond); // Every loop, in every driving mode.

        boolean wasAiming = aiming;
        aiming = false;
        feederState = ScoringIntakeState.INIT;
        intake.setState(opMode.gamepad1.x ? Intake.IntakeState.FORWARD : Intake.IntakeState.BACKWARD);

        // Priority: reverse > manual shot > automatic shot > normal manual driving.
        if (opMode.gamepad1.x) {
            manualDrive();
            feederState = ScoringIntakeState.REVERSE;
            mode = "Reverse";
        } else if (opMode.gamepad2.left_trigger > ShootingConfig.triggerThreshold) {
            manualDrive();
            feederState = ScoringIntakeState.SHOOT;
            mode = "Manual shot";
        } else if (opMode.gamepad2.right_trigger > ShootingConfig.triggerThreshold) {
            aiming = true;
            if (!wasAiming) drive.beginAim();
            drive.driveToHeading(shot.headingRadians);
            if (shot.ready(shooter.getVelocity())) feederState = ScoringIntakeState.SHOOT;
            mode = "Auto aim";
        } else {
            manualDrive();
            mode = "Manual drive";
        }
        feeder.setScoringIntakeState(feederState);
    }

    public void resetPose(Pose pose) {
        drive.setPose(pose);
        aiming = false;
    }

    private void manualDrive() {
        double scale = opMode.gamepad1.left_bumper ? 0.5 : 1.0;
        drive.setDrivePower(opMode.gamepad1.left_stick_x * scale,
                -opMode.gamepad1.left_stick_y * scale, opMode.gamepad1.right_stick_x * scale);
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
        opMode.telemetry.addData("Mode / feeder", "%s / %s", mode, feederState);
        opMode.telemetry.addData("Pose reset", "Driver Y: corner pose");
        opMode.telemetry.addData("Auto ready", shot.ready(actual));
        opMode.telemetry.addData("Warning", shot.warning == null ? "None" : shot.warning);
        opMode.telemetry.addData("PIDF", shooter.isPidfValid() ? "Applied" : "Invalid edit; keeping last gains");
        opMode.telemetry.update();
    }

    public void stopAll() {
        drive.stop();
        shooter.setVelocity(0);
        feeder.setScoringIntakeState(ScoringIntakeState.INIT);
        intake.setState(Intake.IntakeState.INIT);
    }
}
