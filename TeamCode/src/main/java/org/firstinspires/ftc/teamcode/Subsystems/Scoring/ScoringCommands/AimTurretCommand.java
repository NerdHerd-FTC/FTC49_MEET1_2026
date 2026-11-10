package org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShotTargeting;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.TurretSubsystem;

/** Default turret command: continuously point at the selected target while the driver drives. */
public class AimTurretCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final Supplier<ShotTargeting.Solution> solution;

    public AimTurretCommand(TurretSubsystem turret, Supplier<ShotTargeting.Solution> solution) {
        this.turret = turret;
        this.solution = solution;
        addRequirements(turret);
    }

    @Override public void execute() { turret.setAngle(solution.get().turretRadians); }
}
