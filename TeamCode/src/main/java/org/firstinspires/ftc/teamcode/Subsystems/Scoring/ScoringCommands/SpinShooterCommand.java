package org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringSubsystem;

public class SpinShooterCommand extends CommandBase {
    private GoBildaPinpointDriver odo;
    private MyRobot robot;
    private final ScoringSubsystem shooterSubsystem;
    private double shooterPower;
    double RED_GOAL_X = 13.63;
    double RED_GOAL_Y = 127.64;

    public SpinShooterCommand(ScoringSubsystem subsystem, MyRobot robot){
        this.shooterSubsystem = subsystem;
        odo = robot.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(2.935520382378045, -0.567464790944978, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){

    }

    public void execute(){
        Pose2D robotPose = odo.getPosition();
        double y = robotPose.getX(DistanceUnit.INCH);
        double x = robotPose.getY(DistanceUnit.INCH);
        double H = robotPose.getHeading(AngleUnit.RADIANS);
        double robotH = -(H);
        if (y > 72) {
            RED_GOAL_X = 57.76;
            RED_GOAL_Y = 85.84;
        } else {
            RED_GOAL_X = 57.76;
            RED_GOAL_Y = 53.71;
        }

        double dx = RED_GOAL_X - x;
        double dy = RED_GOAL_Y - y;
        double floorDistance = Math.hypot(dx, dy);

        shooterPower = 950 + (floorDistance - 70) * ((1000.0 - 950.0) / (59.55 - 53.5));

        shooterSubsystem.setVelocity(shooterPower);
    }

    @Override
    public boolean isFinished(){
        return  false;
    }

    @Override
    public void end(boolean interrupted){

    }
}
