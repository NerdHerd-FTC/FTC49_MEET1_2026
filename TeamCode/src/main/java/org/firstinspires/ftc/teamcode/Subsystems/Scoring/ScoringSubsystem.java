package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;
@Configurable
public class ScoringSubsystem extends SubsystemBase {
    public DcMotorEx Shooter;
    public DcMotor IntakeBack;
    public double INIT = 0;
    public static double FORWARD = 950;
    public static double BACKWARD = -950;
    public static double kp = 150;// 170
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;// 0

    public enum ShooterState{
        INIT,
        FORWARD,
        BACKWARD
    }

    public ScoringSubsystem (HardwareMap hardwareMap){
        this.Shooter = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        this.Shooter.setVelocityPIDFCoefficients(kp,ki,kd,kf);
    }
    public void setVelocity (double velocity) {Shooter.setVelocity(velocity);}
    public double getVelocity (){
        return Shooter.getVelocity();
    }

    public void setShooterState (ShooterState state) {
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
        }
        Shooter.setVelocity(vel);
    }

}
