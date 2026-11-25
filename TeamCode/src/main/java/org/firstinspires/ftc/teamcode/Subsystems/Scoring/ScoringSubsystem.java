package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;
@Configurable
public class ScoringSubsystem extends SubsystemBase {
    public DcMotorEx ShooterLeft;
    public CRServo ShooterServo1;
    public double INIT = 0;

    public static double FORWARD = 750;
    public static double TEST = 700;
    public static double AUTOFORWARD = 755;
    public static double BACKWARD = -970;
    public static double kp = 185;// 170
    public static double ki = 0;
    public static double kd = 0.001;
    public static double kf = 0;// 0

    public enum ShooterState{
        INIT,
        FORWARD,
        AUTOFORWARD,
        BACKWARD,
        TEST
    }

    public ScoringSubsystem (HardwareMap hardwareMap){
        this.ShooterLeft = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        this.ShooterLeft.setVelocityPIDFCoefficients(kp,ki,kd,kf);
        this.ShooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        this.ShooterServo1 = hardwareMap.get(CRServo.class, "ShooterServo");
        this.ShooterServo1.setDirection(CRServo.Direction.FORWARD);

//        this.ShooterServo2 = hardwareMap.get(CRServo.class, "ShooterServo2");
//        this.ShooterServo2.setDirection(CRServo.Direction.REVERSE);
    }
    public void setServoPower (double power){
        ShooterServo1.setPower(power);
//        ShooterServo2.setPower(power);
    }
    public void changePollen (double change) {
        TEST += change;
    }

    public void changeNectar (double change) {
        FORWARD += change;
    }
    public double getVelocity (){
        return ShooterLeft.getVelocity();
    }
    public double getPollenPower() {return TEST;}
    public double getNectarPower() {return FORWARD;}

    public void setShooterState (ShooterState state) { //Look into follow of shooter motors
        double vel = 0;
        switch (state) {
            case INIT:
                vel = INIT;
                break;
            case FORWARD:
                vel = FORWARD;
                break;
            case BACKWARD:
                vel = BACKWARD;
                break;
            case AUTOFORWARD:
                vel = AUTOFORWARD;
                break;
            case TEST:
                vel = TEST;
                break;
        }
        ShooterLeft.setVelocity(vel);
    }

}
