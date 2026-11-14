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
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.GoalTracker;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands.AimTurretCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem.ScoringIntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShotTargeting;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

public class MyRobot extends Robot {
    private final LinearOpMode opMode;
    private final Alliance alliance;
    private final DriveSubsystem drive;
    private final Intake intake;
    private final ScoringSubsystem shooter;
    private final ScoringIntakeSubsystem feeder;
    private final TurretSubsystem turret;
    private final GoalTracker goals = new GoalTracker();
    private final boolean fromSavedPose;
    private ShotTargeting.Solution shot;
    private String mode = "Manual drive";

    /**
     * Starts from this alliance's last saved pose (auto end, or the previous TeleOp run), else the
     * alliance start pose. The turret encoder is zeroed only if no OpMode has zeroed it yet; then
     * the turret must sit at turretStartDeg at INIT.
     */
    public MyRobot(LinearOpMode opMode, Alliance alliance) {
        reset();
        enable();
        this.opMode = opMode;
        this.alliance = alliance;
        Pose saved = alliance.savedPose();
        fromSavedPose = saved != null;
        drive = new DriveSubsystem(opMode.hardwareMap, fromSavedPose ? saved : alliance.startPose(),
                alliance.driveOffsetRadians());
        intake = new Intake(opMode.hardwareMap);
        shooter = new ScoringSubsystem(opMode.hardwareMap);
        feeder = new ScoringIntakeSubsystem(opMode.hardwareMap);
        turret = new TurretSubsystem(opMode.hardwareMap, !PoseStorage.turretZeroed);

        drive.setDefaultCommand(new DefaultDriveCommand(drive,
                () -> opMode.gamepad1.left_stick_x, () -> -opMode.gamepad1.left_stick_y,
                () -> opMode.gamepad1.right_stick_x, () -> opMode.gamepad1.left_bumper));
        shooter.setDefaultCommand(new SpinShooterCommand(shooter, () -> shot.velocityTicksPerSecond));
        // Turret always tracks the auto-picked goal; no button needed and the drive is never turned.
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

    /** Picks the nearer of the alliance's two goals, then aims and sets speed for it. */
    public void update() {
        drive.updatePose();
        Pose pose = drive.getPose();
        // Saved every loop so the next TeleOp run resumes from here, not from an old auto pose.
        alliance.savePose(pose);
        // Goals are rebuilt every loop so Panels edits apply live.
        shot = goals.update(pose, alliance.leftGoal(), alliance.rightGoal());
        // FTCLib applies live PIDF in periodic(), polls buttons, and runs the commands.
        run();
        mode = opMode.gamepad1.x ? "Reverse" : manualShotRequested() ? "Manual shot"
                : "Manual drive";
    }

    /** Driver Y: snap to the measured corner pose. Returns false while it is still unset. */
    public boolean resetPose() {
        Pose pose = alliance.resetPose();
        if (pose == null) return false;
        drive.setPose(pose);
        return true;
    }

    /** INIT screen: where the robot must be placed and how the turret is zeroed. */
    public void showInitTelemetry() {
        Pose pose = drive.getPose();
        opMode.telemetry.addData("TeleOp", "%s FIELD", alliance);
        opMode.telemetry.addData("Start pose (in, deg)", "%.1f, %.1f, %.0f (%s)", pose.x(), pose.y(),
                Math.toDegrees(pose.heading()), fromSavedPose ? "saved from last OpMode" : "alliance start: place robot there");
        opMode.telemetry.addData("Turret", "angle %.1f deg (%s)", Math.toDegrees(turret.getAngle()),
                PoseStorage.turretZeroed ? "zero kept from earlier OpMode" : "zeroed now");
        opMode.telemetry.update();
    }

    private boolean manualShotRequested() {
        return opMode.gamepad2.left_trigger > 0;
    }

    public void showTelemetry() {
        Pose pose = drive.getPose();
        double actual = shooter.getVelocity();
        opMode.telemetry.addData("TeleOp / auto goal", "%s / %s", alliance, goals.isLeft() ? "LEFT" : "RIGHT");
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
        opMode.telemetry.addData("Pose reset", alliance.resetPose() == null
                ? "Driver Y disabled: reset pose not measured" : "Driver Y: corner pose");
        // Parked at a travel limit is not "on target" when the goal is out of reach.
        opMode.telemetry.addData("Turret on target", turret.onTarget() && shot.turretReachable);
        opMode.telemetry.addData("Warning", shot.warning == null ? "None" : shot.warning);
        opMode.telemetry.addData("PIDF", shooter.isPidfValid() ? "Applied" : "Invalid edit; keeping last gains");
        opMode.telemetry.update();
    }

    public void stopAll() {
        reset();
        alliance.savePose(drive.getPose());
        drive.stop();
        turret.stop();
        shooter.setVelocity(0);
        feeder.setScoringIntakeState(ScoringIntakeState.INIT);
        intake.setState(Intake.IntakeState.INIT);
    }
}
