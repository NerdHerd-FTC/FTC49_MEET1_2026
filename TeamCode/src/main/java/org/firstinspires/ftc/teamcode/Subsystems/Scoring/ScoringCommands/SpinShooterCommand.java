package org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;

public class SpinShooterCommand extends CommandBase {
    private final ScoringSubsystem shooterSubsystem;
    private final ScoringSubsystem.ShooterState targetState;

    public SpinShooterCommand(ScoringSubsystem subsystem, ScoringSubsystem.ShooterState inputState){
        this.shooterSubsystem = subsystem;
        this.targetState = inputState;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.setShooterState(targetState);
    }

    public void execute(){
        //nothing to do in the loop
    }

    @Override
    public boolean isFinished(){
        return  false;
    }

    @Override
    public void end(boolean interrupted){

    }
}
