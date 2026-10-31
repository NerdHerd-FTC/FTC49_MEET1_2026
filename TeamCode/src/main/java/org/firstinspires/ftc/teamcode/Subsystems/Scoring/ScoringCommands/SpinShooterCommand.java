package org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;

public class SpinShooterCommand extends CommandBase {
    private final ScoringSubsystem shooter;
    private final DoubleSupplier velocity;

    public SpinShooterCommand(ScoringSubsystem shooter, DoubleSupplier velocity) {
        this.shooter = shooter;
        this.velocity = velocity;
        addRequirements(shooter);
    }

    @Override public void initialize() { execute(); }

    @Override public void execute() { shooter.setVelocity(velocity.getAsDouble()); }

    @Override public void end(boolean interrupted) { shooter.setVelocity(0); }
}
