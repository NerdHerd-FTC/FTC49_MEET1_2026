package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;

@Configurable
public class ScoringIntakeSubsystem extends SubsystemBase {
    public DcMotor IntakeBack;
    public CRServo IndexServo;
    public double[] INIT = {0,0};
    public static double[] POLLEN = {-1, -1};
    public static double[] NECTAR = {-0.75, -0.5};
    public static double[] SLOWTAR = {-0.5, -0.5};
    public static double[] REVERSE = {0.5, 0};
    public static double[] AUTO = {-0.5, -1};


    public enum ScoringIntakeState{
        INIT,
        POLLEN,
        NECTAR,
        SLOWTAR,
        REVERSE,
        AUTO
    }

    public ScoringIntakeSubsystem(HardwareMap hardwareMap){
        this.IntakeBack = hardwareMap.get(DcMotor.class, "IntakeFront");
        IntakeBack.setDirection(DcMotorSimple.Direction.FORWARD);

        this.IndexServo = hardwareMap.get(CRServo.class, "IndexServo");
        IndexServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setScoringIntakeState (ScoringIntakeState state) {
        double[] vel = {};
        switch (state) {
            case INIT:
                vel = INIT;
                break;
            case POLLEN:
                vel = POLLEN;
                break;
            case NECTAR:
                vel = NECTAR;
                break;
            case REVERSE:
                vel = REVERSE;
                break;
            case AUTO:
                vel = AUTO;
                break;
            case SLOWTAR:
                vel = SLOWTAR;
                break;
        }
        IntakeBack.setPower(vel[0]);
        IndexServo.setPower(vel[1]);
    }
}
