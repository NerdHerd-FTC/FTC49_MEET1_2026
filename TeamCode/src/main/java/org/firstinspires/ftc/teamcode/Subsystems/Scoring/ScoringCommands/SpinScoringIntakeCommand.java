package org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;

public class SpinScoringIntakeCommand extends CommandBase {
    private final ScoringIntakeSubsystem intakeSubsystem;
    private final ScoringIntakeSubsystem.ScoringIntakeState targetState;

    public SpinScoringIntakeCommand(ScoringIntakeSubsystem subsystem, ScoringIntakeSubsystem.ScoringIntakeState inputState){
        this.intakeSubsystem = subsystem;
        this.targetState = inputState;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.setScoringIntakeState(targetState);
    }

    public void execute(){
        //nothing to do in the loop
    }

    @Override
    public boolean isFinished(){
        return  true;
    }

    @Override
    public void end(boolean interrupted){

    }
}