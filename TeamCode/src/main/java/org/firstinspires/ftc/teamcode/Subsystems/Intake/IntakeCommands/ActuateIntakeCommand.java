package org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;

public class ActuateIntakeCommand extends CommandBase {
    private final Intake IntakeSubsystem;
    private final Intake.IntakeState targetState;

    public ActuateIntakeCommand(Intake subsystem, Intake.IntakeState inputState){
        this.IntakeSubsystem = subsystem;
        this.targetState = inputState;
        addRequirements(IntakeSubsystem);
    }

    @Override
    public void initialize(){
        IntakeSubsystem.setState(targetState);
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
