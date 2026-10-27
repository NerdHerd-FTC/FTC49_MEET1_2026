package org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveCommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class  SlowModeCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;

    private static double DRIVE_SLOW_SCALE = 0.5;
    private static double STRAFE_SLOW_SCALE = 0.5;
    private static double ROT_SLOW_SCALE = 0.5;

    public SlowModeCommand(DriveSubsystem subsystem, DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier turnInput) {
        this.driveSubsystem = subsystem;
        driveSupplier = driveInput;
        strafeSupplier = strafeInput;
        turnSupplier = turnInput;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        driveSubsystem.setDrivePower(driveSupplier.getAsDouble() * DRIVE_SLOW_SCALE, strafeSupplier.getAsDouble() * STRAFE_SLOW_SCALE, turnSupplier.getAsDouble() * ROT_SLOW_SCALE);
    }
}