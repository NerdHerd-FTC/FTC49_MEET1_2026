package org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem.ScoringIntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShotTargeting;

public class AimAndShootCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final ScoringIntakeSubsystem feeder;
    private final Supplier<ShotTargeting.Solution> solution;
    private final DoubleSupplier shooterVelocity;

    public AimAndShootCommand(DriveSubsystem drive, ScoringIntakeSubsystem feeder,
            Supplier<ShotTargeting.Solution> solution, DoubleSupplier shooterVelocity) {
        this.drive = drive;
        this.feeder = feeder;
        this.solution = solution;
        this.shooterVelocity = shooterVelocity;
        // Leave the shooter free to run its dynamic-speed default command.
        addRequirements(drive, feeder);
    }

    @Override public void initialize() { drive.beginAim(); }

    @Override public void execute() {
        ShotTargeting.Solution shot = solution.get();
        drive.driveToHeading(shot.headingRadians);
        feeder.setScoringIntakeState(shot.ready(shooterVelocity.getAsDouble())
                ? ScoringIntakeState.SHOOT : ScoringIntakeState.INIT);
    }

    @Override public void end(boolean interrupted) {
        drive.stop();
        feeder.setScoringIntakeState(ScoringIntakeState.INIT);
    }
}
