package org.firstinspires.ftc.teamcode.Subsystems.Robot;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveCommand.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveCommand.SlowModeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlowerIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeCommands.ActuateFlowerCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeCommands.ActuateIntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.GoalTracker;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands.AimTurretCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands.SpinScoringIntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShotTargeting;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

import java.util.function.BooleanSupplier;

public class MyRobot extends Robot {
    DriveSubsystem drive;
    Intake intake;
    ScoringSubsystem shooter;
    ScoringIntakeSubsystem scoringIntake;
    FlowerIntake flowerIntake;
    TurretSubsystem turretSubsystem;
    DefaultDriveCommand defaultDriveCommand;
    SlowModeCommand slowModeCommand;
    Pose saved;

    public LinearOpMode opMode;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public GamepadEx driver;
    public GamepadEx operator;
    private final GoalTracker goals = new GoalTracker();
    private final boolean fromSavedPose;
    private ShotTargeting.Solution shot;
    private String mode = "Manual drive";
    private final Alliance alliance;
    public boolean manual = false;


    public enum TeleopMode {
        FIELD,
        ROBOT,
        TEST
    }
    public MyRobot (LinearOpMode opMode, TeleopMode mode, Alliance alliance) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.driver = new GamepadEx(opMode.gamepad1);
        this.operator = new GamepadEx(opMode.gamepad2);
        this.alliance = alliance;
        this.saved = alliance.savedPose();
        fromSavedPose = saved != null;
        initTele(mode);
    }

    public void initTele (TeleopMode mode){
        if (mode == TeleopMode.FIELD){
            reset();
            enable();
            drive = new DriveSubsystem(this.hardwareMap, fromSavedPose ? saved : alliance.startPose(),
                    alliance.driveOffsetRadians());

            intake = new Intake(this.hardwareMap);
            shooter = new ScoringSubsystem(this.hardwareMap);
            scoringIntake = new ScoringIntakeSubsystem(this.hardwareMap);
            flowerIntake = new FlowerIntake(this.hardwareMap);
            turretSubsystem = new TurretSubsystem(this.hardwareMap, !PoseStorage.turretZeroed);
            drive.setFieldOriented(true);
            defaultDriveCommand = new DefaultDriveCommand(drive,
                    driver::getLeftX,
                    driver::getLeftY,
                    driver::getRightX
            );

            slowModeCommand = new SlowModeCommand(drive,
                    driver::getLeftX,
                    driver::getLeftY,
                    driver::getRightX
            );
            flowerIntake.setState(FlowerIntake.FlowerState.RETRACT);
            CommandScheduler.getInstance().setDefaultCommand(drive, defaultDriveCommand);
            shooter.setShooterState(ScoringSubsystem.ShooterState.FORWARD);
            intake.setState(IntakeState.FORWARD);
            turretSubsystem.setDefaultCommand(
                    new AimTurretCommand(
                            turretSubsystem,
                            () -> shot,
                            operator :: getRightX,
                            manual));
            Button driverSlowMode = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
            Button shooterStart = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
            Button intakeRestart = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
            Button intakeReverse = new GamepadButton(driver, GamepadKeys.Button.X);
            Button flowerStart = new GamepadButton(driver, GamepadKeys.Button.Y);
            Button testStart = new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER);
            Button driverCalibrate = new GamepadButton(driver, GamepadKeys.Button.B);

            driverSlowMode
                    .whenPressed(slowModeCommand)
                    .whenReleased(defaultDriveCommand);
            
            /*autoDrive
                    .whenPressed(new AutoDriveCommand(drive))
                            .whenReleased(defaultDriveCommand);*/
            shooterStart
                    .whenPressed(
                            new ParallelCommandGroup
                                    (
                                            new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.FORWARD),
                                            new ActuateIntakeCommand(intake, IntakeState.NECTAR),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.NECTAR)
                                    ))
                            .whenReleased(new ParallelCommandGroup(
                                    new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.FORWARD),
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT),
                                    new ActuateIntakeCommand(intake, IntakeState.FORWARD)
                            ));

            intakeReverse
                    .whenPressed(
                            new ParallelCommandGroup
                                    (
                                            new ActuateIntakeCommand(intake, IntakeState.BACKWARD),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.REVERSE)
                                    ))
                    .whenReleased(
                            new ParallelCommandGroup
                                    (
                                            new ActuateIntakeCommand(intake, IntakeState.FORWARD),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT)
                                    ));

            flowerStart
                    .whenPressed(new ActuateFlowerCommand(flowerIntake, FlowerIntake.FlowerState.INTAKE))
                    .whenReleased(new ActuateFlowerCommand(flowerIntake, FlowerIntake.FlowerState.RETRACT));


            testStart
                    .whenPressed(
                            new SequentialCommandGroup
                                    (
                                            new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.TEST),
                                            new WaitCommand(500),
                                            new ActuateIntakeCommand(intake, IntakeState.POLLEN),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.POLLEN)
                                    ))
                    .whenReleased(
                            new ParallelCommandGroup(
                                    new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.FORWARD),
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT),
                                    new ActuateIntakeCommand(intake, IntakeState.FORWARD)
                            )
                    );
            driverCalibrate.whenPressed(
                    new InstantCommand(()-> drive.setPose(alliance.resetPose))
            );
            /*shooterStop
                    .whenPressed(new ParallelCommandGroup
                            (
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT)
                            ));*/

        }if (mode == TeleopMode.ROBOT){
            drive = new DriveSubsystem(opMode.hardwareMap, fromSavedPose ? saved : alliance.startPose(),
                    alliance.driveOffsetRadians());
            intake = new Intake(this.hardwareMap);
            shooter = new ScoringSubsystem(this.hardwareMap);
            scoringIntake = new ScoringIntakeSubsystem(this.hardwareMap);
            flowerIntake = new FlowerIntake(this.hardwareMap);
            drive.setFieldOriented(false);
            update();
            reset();
            enable();
            defaultDriveCommand = new DefaultDriveCommand(drive,
                    driver::getLeftX,
                    driver::getLeftY,
                    driver::getRightX
            );

            slowModeCommand = new SlowModeCommand(drive,
                    driver::getLeftX,
                    driver::getLeftY,
                    driver::getRightX
            );
            flowerIntake.setState(FlowerIntake.FlowerState.RETRACT);
            CommandScheduler.getInstance().setDefaultCommand(drive, defaultDriveCommand);
            shooter.setShooterState(ScoringSubsystem.ShooterState.FORWARD);
            intake.setState(IntakeState.FORWARD);
            Button driverSlowMode = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
            Button shooterStart = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
            Button intakeRestart = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
            Button testStart = new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER);
            Button intakeReverse = new GamepadButton(driver, GamepadKeys.Button.X);
            Button flowerStart = new GamepadButton(driver, GamepadKeys.Button.Y);

            driverSlowMode
                    .whenPressed(slowModeCommand)
                    .whenReleased(defaultDriveCommand);

            /*autoDrive
                    .whenPressed(new AutoDriveCommand(drive))
                            .whenReleased(defaultDriveCommand);*/
            shooterStart
                    .whenPressed(
                            new ParallelCommandGroup
                                    (
                                            new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.FORWARD),
                                            new ActuateIntakeCommand(intake, IntakeState.NECTAR),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.NECTAR)
                                    ))
                    .whenReleased(
                            new ParallelCommandGroup(
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT),
                                    new ActuateIntakeCommand(intake, IntakeState.FORWARD)
                            )
                    );

            intakeReverse
                    .whenPressed(
                            new ParallelCommandGroup
                                    (
                                            new ActuateIntakeCommand(intake, IntakeState.BACKWARD),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.REVERSE)
                                    ))
                    .whenReleased(
                            new ParallelCommandGroup
                                    (
                                            new ActuateIntakeCommand(intake, IntakeState.FORWARD),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT)
                                    ));

            flowerStart
                    .whenPressed(new ActuateFlowerCommand(flowerIntake, FlowerIntake.FlowerState.INTAKE))
                    .whenReleased(new ActuateFlowerCommand(flowerIntake, FlowerIntake.FlowerState.RETRACT));

            testStart
                    .whenPressed(
                            new ParallelCommandGroup
                                    (
                                            new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.TEST),
                                            new WaitCommand(500),
                                            new ActuateIntakeCommand(intake, IntakeState.POLLEN),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.POLLEN)
                                    ))
                    .whenReleased(
                            new ParallelCommandGroup(
                                    new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.FORWARD),
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT),
                                    new ActuateIntakeCommand(intake, IntakeState.FORWARD)
                            )
                    );


        } else if (mode == TeleopMode.TEST){
            reset();
            enable();
            drive = new DriveSubsystem(this.hardwareMap, fromSavedPose ? saved : alliance.startPose(),
                    alliance.driveOffsetRadians());

            intake = new Intake(this.hardwareMap);
            shooter = new ScoringSubsystem(this.hardwareMap);
            scoringIntake = new ScoringIntakeSubsystem(this.hardwareMap);
            flowerIntake = new FlowerIntake(this.hardwareMap);
            turretSubsystem = new TurretSubsystem(this.hardwareMap, !PoseStorage.turretZeroed);
            drive.setFieldOriented(true);
            defaultDriveCommand = new DefaultDriveCommand(drive,
                    driver::getLeftX,
                    driver::getLeftY,
                    driver::getRightX
            );

            slowModeCommand = new SlowModeCommand(drive,
                    driver::getLeftX,
                    driver::getLeftY,
                    driver::getRightX
            );
            flowerIntake.setState(FlowerIntake.FlowerState.RETRACT);
            CommandScheduler.getInstance().setDefaultCommand(drive, defaultDriveCommand);
            shooter.setShooterState(ScoringSubsystem.ShooterState.FORWARD);
            intake.setState(IntakeState.FORWARD);
            turretSubsystem.setDefaultCommand(
                    new AimTurretCommand(
                            turretSubsystem,
                            () -> shot,
                            operator :: getRightX,
                            manual));

            Button driverSlowMode = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
            Button shooterStart = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
            Button intakeReverse = new GamepadButton(driver, GamepadKeys.Button.X);
            Button flowerStart = new GamepadButton(driver, GamepadKeys.Button.Y);
            Button testStart = new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER);
            Button driverCalibrate = new GamepadButton(driver, GamepadKeys.Button.B);
            Button pollenIncrease = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
            Button pollenDecrease = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);
            Button nectarIncrease = new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT);
            Button nectarDecrease = new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT);
            Button turretManual = new GamepadButton(operator, GamepadKeys.Button.X);
            Button turretAuto = new GamepadButton(operator, GamepadKeys.Button.Y);
            Button slowNectar = new GamepadButton(operator, GamepadKeys.Button.A);

            driverSlowMode
                    .whenPressed(slowModeCommand)
                    .whenReleased(defaultDriveCommand);

            /*autoDrive
                    .whenPressed(new AutoDriveCommand(drive))
                            .whenReleased(defaultDriveCommand);*/
            shooterStart
                    .whenPressed(
                            new ParallelCommandGroup
                                    (
                                            new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.FORWARD),
                                            new ActuateIntakeCommand(intake, IntakeState.NECTAR),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.NECTAR)
                                    ))
                    .whenReleased(new ParallelCommandGroup(
                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT),
                            new ActuateIntakeCommand(intake, IntakeState.FORWARD)
                    ));

            intakeReverse
                    .whenPressed(
                            new ParallelCommandGroup
                                    (
                                            new ActuateIntakeCommand(intake, IntakeState.BACKWARD),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.REVERSE)
                                    ))
                    .whenReleased(
                            new ParallelCommandGroup
                                    (
                                            new ActuateIntakeCommand(intake, IntakeState.FORWARD),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT)
                                    ));

            flowerStart
                    .whenPressed(new ActuateFlowerCommand(flowerIntake, FlowerIntake.FlowerState.INTAKE))
                    .whenReleased(new ActuateFlowerCommand(flowerIntake, FlowerIntake.FlowerState.RETRACT));


            testStart
                    .whenPressed(
                            new SequentialCommandGroup
                                    (
                                            new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.TEST),
                                            new WaitCommand(500),
                                            new ActuateIntakeCommand(intake, IntakeState.POLLEN),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.POLLEN)
                                    ))
                    .whenReleased(
                            new ParallelCommandGroup(
                                    new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.FORWARD),
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT),
                                    new ActuateIntakeCommand(intake, IntakeState.FORWARD)
                            )
                    );

            slowNectar
                    .whenPressed(
                            new SequentialCommandGroup
                                    (
                                            new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.TEST),
                                            new ActuateIntakeCommand(intake, IntakeState.POLLEN),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.SLOWTAR)
                                    ))
                    .whenReleased(
                            new ParallelCommandGroup(
                                    new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.FORWARD),
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT),
                                    new ActuateIntakeCommand(intake, IntakeState.FORWARD)
                            )
                    );

            driverCalibrate.whenPressed(
                    new InstantCommand(()-> drive.setPose(alliance.resetPose))
            );
            pollenIncrease.whenPressed(
                    new InstantCommand(()-> shooter.changePollen(10))
            );
            pollenDecrease.whenPressed(
                    new InstantCommand(()-> shooter.changePollen(-10))
            );
            nectarIncrease.whenPressed(
                    new InstantCommand(()-> shooter.changeNectar(10))
            );
            nectarDecrease.whenPressed(
                    new InstantCommand(()-> shooter.changeNectar(-10))
            );
            turretManual.whenPressed(
                    new InstantCommand(()-> manual = true)
            );
            turretAuto.whenPressed(
                    new InstantCommand(()-> manual = false)
            );

            /*shooterStop
                    .whenPressed(new ParallelCommandGroup
                            (
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT)
                            ));*/
        }
    }
    public void update() {
        drive.update();
        Pose pose = drive.getPose();
        // Saved every loop so the next TeleOp run resumes from here, not from an old auto pose.
        alliance.savePose(pose);
        // Goals are rebuilt every loop so Panels edits apply live.
        shot = goals.update(pose, alliance.leftGoal(), alliance.rightGoal());
        // FTCLib applies live PIDF in periodic(), polls buttons, and runs the commands.
        opMode.telemetry.addData("ShooterPower", shooter.getVelocity());
        opMode.telemetry.addData("PollenPower", shooter.getPollenPower());
        opMode.telemetry.addData("NectarPower", shooter.getNectarPower());
        opMode.telemetry.addData("Manual: ", manual);
        opMode.telemetry.update();
        run();
    }


    /** Driver Y: snap to the measured corner pose. Returns false while it is still unset. */
    public boolean resetPose() {
        Pose pose = alliance.resetPose();
        if (pose == null) return false;
        drive.setPose(pose);
        return true;
    }


}
