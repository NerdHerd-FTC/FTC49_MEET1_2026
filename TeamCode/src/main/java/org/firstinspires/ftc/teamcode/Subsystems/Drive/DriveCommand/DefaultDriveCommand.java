package org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveCommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;

    public DefaultDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier turnInput) {
        this.driveSubsystem = driveSubsystem;
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
        driveSubsystem.setDrivePower(driveSupplier.getAsDouble(), strafeSupplier.getAsDouble(), turnSupplier.getAsDouble());
    }
}