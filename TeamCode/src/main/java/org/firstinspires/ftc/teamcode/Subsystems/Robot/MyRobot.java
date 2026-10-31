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
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeCommands.ActuateIntakeCommand;


public class MyRobot extends Robot {
    Intake intake;

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
            intake = new Intake(this.hardwareMap);


            CommandScheduler.getInstance().setDefaultCommand(intake, new ActuateIntakeCommand(intake, IntakeState.BACKWARD));
            Button driverSlowMode = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
            Button shooterStart = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
            Button intakeRestart = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
            Button intakeReverse = new GamepadButton(driver, GamepadKeys.Button.X);

            
            /*autoDrive
                    .whenPressed(new AutoDriveCommand(drive))
                            .whenReleased(defaultDriveCommand);*/

            intakeReverse
                    .whenPressed(
                            new ParallelCommandGroup
                                    (
                                            new ActuateIntakeCommand(intake, IntakeState.FORWARD)
                                    ))
                    .whenReleased(
                            new ParallelCommandGroup
                                    (
                                            new ActuateIntakeCommand(intake, IntakeState.BACKWARD)
                                    ));

            /*shooterStop
                    .whenPressed(new ParallelCommandGroup
                            (
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT)
                            ));*/

        }if (mode == TeleopMode.ROBOT){
            intake = new Intake(this.hardwareMap);

            CommandScheduler.getInstance().setDefaultCommand(intake, new ActuateIntakeCommand(intake, IntakeState.BACKWARD));
            Button driverSlowMode = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
            Button shooterStart = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
            Button autoDrive = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
            Button shooterReverse = new GamepadButton(operator, GamepadKeys.Button.A);
            Button intakeShoot = new GamepadButton(driver, GamepadKeys.Button.X);
            Button intakeReverse = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);


            /*autoDrive
                    .whenPressed(new AutoDriveCommand(drive))
                            .whenReleased(defaultDriveCommand);*/
            /*shooterStart
                    .whenPressed(
                            new ParallelCommandGroup
                                    (
                                            new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.FORWARD),
                                            new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.SHOOT)
                                    ))
                    .whenReleased(new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT));

            shooterReverse
                    .whenPressed(new SpinShooterCommand(shooter, ScoringSubsystem.ShooterState.BACKWARD))
                    .whenReleased(new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT));;

            /*shooterStop
                    .whenPressed(new ParallelCommandGroup
                            (
                                    new SpinScoringIntakeCommand(scoringIntake, ScoringIntakeSubsystem.ScoringIntakeState.INIT)
                            ));*/

        }
    }

}
