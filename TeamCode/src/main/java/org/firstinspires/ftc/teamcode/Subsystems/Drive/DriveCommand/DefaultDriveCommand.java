package org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final DoubleSupplier leftX, leftY, rightX;
    private final BooleanSupplier slow;

    public DefaultDriveCommand(DriveSubsystem drive, DoubleSupplier leftX,
            DoubleSupplier leftY, DoubleSupplier rightX, BooleanSupplier slow) {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.slow = slow;
        addRequirements(drive);
    }

    @Override public void initialize() { execute(); }

    @Override public void execute() {
        double scale = slow.getAsBoolean() ? 0.5 : 1.0;
        drive.setDrivePower(leftX.getAsDouble() * scale, leftY.getAsDouble() * scale,
                rightX.getAsDouble() * scale);
    }

    @Override public void end(boolean interrupted) { drive.stop(); }
}
