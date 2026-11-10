package org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.FlowerIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;

public class ActuateFlowerCommand extends CommandBase {
    private final FlowerIntake FlowerIntake;
    private final FlowerIntake.FlowerState targetState;

    public ActuateFlowerCommand(FlowerIntake subsystem, FlowerIntake.FlowerState inputState){
        this.FlowerIntake = subsystem;
        this.targetState = inputState;
        addRequirements(FlowerIntake);
    }

    @Override
    public void initialize(){
        FlowerIntake.setState(targetState);
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
