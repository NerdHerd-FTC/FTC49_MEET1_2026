package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;

public class FlowerIntake extends SubsystemBase{
    public MyRobot robot;
    public Servo FlowerLeft;
    public Servo FlowerRight;
    public double INIT = 0;

    public double INTAKE = 0.725;
    public double RETRACT = 0.325;

    public enum FlowerState{
        INIT,
        INTAKE,
        RETRACT
    }

    public FlowerIntake(HardwareMap  hardwareMap){
        this.FlowerLeft = hardwareMap.get(Servo.class, "FlowerLeft");
        this.FlowerLeft.setDirection(Servo.Direction.REVERSE);

        this.FlowerRight = hardwareMap.get(Servo.class, "FlowerRight");
    }

    public void setState(FlowerState state) {
        double pos = 0;
        switch (state) {
            case INIT:
                pos = INIT;
                break;
            case INTAKE:
                pos = INTAKE;
                break;
            case RETRACT:
                pos = RETRACT;
                break;
        }
        FlowerLeft.setPosition(pos);
        FlowerRight.setPosition(pos);
    }
}
