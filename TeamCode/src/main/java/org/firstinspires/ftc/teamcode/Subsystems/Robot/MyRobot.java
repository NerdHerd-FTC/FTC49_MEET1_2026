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
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands.AimTurretCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem.ScoringIntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShotTargeting;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.TurretSubsystem;

public class MyRobot extends Robot {
    private final LinearOpMode opMode;
    private final DriveSubsystem drive;
    private final Intake intake;
    private final ScoringSubsystem shooter;
    private final ScoringIntakeSubsystem feeder;
    private final TurretSubsystem turret;
    private final ShotTargeting targeting = new ShotTargeting();
    private ShotTargeting.Solution shot;
    private String mode = "Manual drive";

    /** @param zeroTurret true only when the turret sits at turretStartDeg (no auto ran this session). */
    public MyRobot(LinearOpMode opMode, Pose startPose, boolean zeroTurret) {
        reset();
        enable();
        this.opMode = opMode;
        drive = new DriveSubsystem(opMode.hardwareMap, startPose);
        intake = new Intake(opMode.hardwareMap);
        shooter = new ScoringSubsystem(opMode.hardwareMap);
        feeder = new ScoringIntakeSubsystem(opMode.hardwareMap);
        turret = new TurretSubsystem(opMode.hardwareMap, zeroTurret);

        drive.setDefaultCommand(new DefaultDriveCommand(drive,
                () -> opMode.gamepad1.left_stick_x, () -> -opMode.gamepad1.left_stick_y,
                () -> opMode.gamepad1.right_stick_x, () -> opMode.gamepad1.left_bumper));
        shooter.setDefaultCommand(new SpinShooterCommand(shooter, () -> shot.velocityTicksPerSecond));
        // Turret always tracks the selected target; no button needed and the drive is never turned.
        turret.setDefaultCommand(new AimTurretCommand(turret, () -> shot));
        intake.setDefaultCommand(new RunCommand(() -> intake.setState(Intake.IntakeState.BACKWARD), intake));
        feeder.setDefaultCommand(new RunCommand(() -> feeder.setScoringIntakeState(ScoringIntakeState.INIT), feeder));

        // Priority: reverse > shot. Each owns only the hardware it uses.
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
    }

    public void update(double targetX, double targetY, String shots, boolean calibrated) {
        drive.updatePose();
        shot = targeting.calculate(drive.getPose(), targetX, targetY, shots, calibrated);
        // FTCLib applies live PIDF in periodic(), polls buttons, and runs the commands.
        run();
        mode = opMode.gamepad1.x ? "Reverse" : manualShotRequested() ? "Manual shot"
                : "Manual drive";
    }

    public void resetPose(Pose pose) {
        drive.setPose(pose);
    }

    private boolean manualShotRequested() {
        return opMode.gamepad2.left_trigger > 0;
    }

    public void showTelemetry(String alliance, boolean left) {
        Pose pose = drive.getPose();
        double actual = shooter.getVelocity();
        opMode.telemetry.addData("TeleOp / cell", "%s / %s", alliance, left ? "LEFT" : "RIGHT");
        opMode.telemetry.addData("Pedro pose (in, deg)", "%.2f, %.2f, %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        opMode.telemetry.addData("Distance (in)", "%.2f", shot.distanceIn);
        opMode.telemetry.addData("Turret target / commanded / actual (deg)", "%.2f / %.2f / %.2f",
                Math.toDegrees(shot.turretRadians), Math.toDegrees(turret.getCommandedAngle()),
                Math.toDegrees(turret.getAngle()));
        opMode.telemetry.addData("Turret ticks", turret.getTicks());
        opMode.telemetry.addData("Shooter target / actual (ticks/s)", "%.1f / %.1f",
                shooter.getTargetVelocity(), actual);
        opMode.telemetry.addData("Mode / feeder", "%s / %s", mode, feeder.getState());
        opMode.telemetry.addData("Pose reset", "Driver Y: corner pose");
        opMode.telemetry.addData("Turret on target", turret.onTarget());
        opMode.telemetry.addData("Warning", shot.warning == null ? "None" : shot.warning);
        opMode.telemetry.addData("PIDF", shooter.isPidfValid() ? "Applied" : "Invalid edit; keeping last gains");
        opMode.telemetry.update();
    }

    public void stopAll() {
        reset();
        drive.stop();
        turret.stop();
        shooter.setVelocity(0);
        feeder.setScoringIntakeState(ScoringIntakeState.INIT);
        intake.setState(Intake.IntakeState.INIT);
    }
}
