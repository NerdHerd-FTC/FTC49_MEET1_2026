package org.firstinspires.ftc.teamcode.Subsystems.Robot;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveCommand.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveCommand.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveCommand.SlowModeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeCommands.ActuateIntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands.SpinScoringIntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands.SpinShooterCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;

public class MyRobot extends Robot {
    DriveSubsystem drive;
    Intake intake;
    ScoringSubsystem shooter;
    ScoringIntakeSubsystem scoringIntake;
    DefaultDriveCommand defaultDriveCommand;
    SlowModeCommand slowModeCommand;

    public LinearOpMode opMode;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public GamepadEx driver;
    public GamepadEx operator;

    public enum TeleopMode {
        FIELD,
        ROBOT
    }
    public MyRobot (LinearOpMode opMode, TeleopMode mode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.driver = new GamepadEx(opMode.gamepad1);
        this.operator = new GamepadEx(opMode.gamepad2);
        initTele(mode);
    }

    public void initTele (TeleopMode mode){
        if (mode == TeleopMode.FIELD){
            drive = new DriveSubsystem(this);
            intake = new Intake(this.hardwareMap);
            shooter = new ScoringSubsystem(this.hardwareMap);
            scoringIntake = new ScoringIntakeSubsystem(this.hardwareMap);
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

            CommandScheduler.getInstance().setDefaultCommand(drive, defaultDriveCommand);
            CommandScheduler.getInstance().setDefaultCommand(shooter, new SpinShooterCommand(shooter, this));
            CommandScheduler.getInstance().setDefaultCommand(intake, new ActuateIntakeCommand(intake, IntakeState.BACKWARD));
            Button driverSlowMode = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
            Button shooterStart = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
            Button intakeRestart = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
            Button intakeReverse = new GamepadButton(driver, GamepadKeys.Button.X);

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
                                            new SpinShooterCommand(shooter, this),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.SHOOT)
                                    ))
                            .whenReleased(new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT));

            intakeReverse
                    .whenPressed(
                            new ParallelCommandGroup
                                    (
                                            new ActuateIntakeCommand(intake, IntakeState.FORWARD),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.REVERSE)
                                    ))
                    .whenReleased(
                            new ParallelCommandGroup
                                    (
                                            new ActuateIntakeCommand(intake, IntakeState.BACKWARD),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT)
                                    ));

            /*shooterStop
                    .whenPressed(new ParallelCommandGroup
                            (
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT)
                            ));*/

        }if (mode == TeleopMode.ROBOT){
            drive = new DriveSubsystem(this);
            intake = new Intake(this.hardwareMap);
            shooter = new ScoringSubsystem(this.hardwareMap);
            scoringIntake = new ScoringIntakeSubsystem(this.hardwareMap);
            drive.setFieldOriented(false);

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

            CommandScheduler.getInstance().setDefaultCommand(drive, defaultDriveCommand);
            CommandScheduler.getInstance().setDefaultCommand(shooter, new SpinShooterCommand(shooter, this));
            CommandScheduler.getInstance().setDefaultCommand(intake, new ActuateIntakeCommand(intake, IntakeState.BACKWARD));
            Button driverSlowMode = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
            Button shooterStart = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
            Button autoDrive = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
            Button shooterReverse = new GamepadButton(operator, GamepadKeys.Button.A);
            Button intakeShoot = new GamepadButton(driver, GamepadKeys.Button.X);
            Button intakeReverse = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);

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
                                            new SpinShooterCommand(shooter, this),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.SHOOT)
                                    ))
                    .whenReleased(new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT));

            shooterReverse
                    .whenPressed(new SpinShooterCommand(shooter,this))
                    .whenReleased(new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT));;

            /*shooterStop
                    .whenPressed(new ParallelCommandGroup
                            (
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT)
                            ));*/

        }
    }

}
