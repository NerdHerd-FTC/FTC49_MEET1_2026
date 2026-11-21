package org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands;


import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShotTargeting;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.TurretSubsystem;


/** Default turret command: continuously point at the selected target while the driver drives. */
public class AimTurretCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final Supplier<ShotTargeting.Solution> solution;
    private final DoubleSupplier turretSupplier;


    public AimTurretCommand(TurretSubsystem turret, Supplier<ShotTargeting.Solution> solution, DoubleSupplier turretSupplier) {
        this.turret = turret;
        this.solution = solution;
        this.turretSupplier = turretSupplier;
        addRequirements(turret);
    }


    @Override public void execute() {

        boolean b = !(turretSupplier.getAsDouble() > 0.2 || turretSupplier.getAsDouble() < -0.2);
        if (b) {
            turret.setAngle(solution.get().turretRadians);
        } else {
            turret.setPower(-turretSupplier.getAsDouble());
        }
    }
}
