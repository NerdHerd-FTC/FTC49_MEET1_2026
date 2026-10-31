package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class ScoringIntakeSubsystem extends SubsystemBase {
    public DcMotor IntakeBack;
    public DcMotor IntakeFront;
    public double INIT = 0;
    public static double SHOOT = -1;
    public static double REVERSE = 1;
    private ScoringIntakeState state = ScoringIntakeState.INIT;


    public enum ScoringIntakeState{
        INIT,
        SHOOT,
        REVERSE
    }

    public ScoringIntakeSubsystem(HardwareMap hardwareMap){
        this.IntakeBack = hardwareMap.get(DcMotor.class, "IntakeBack");
        this.IntakeFront = hardwareMap.get(DcMotor.class, "IntakeFront");
    }


    public ScoringIntakeState getState() { return state; }

    public void setScoringIntakeState (ScoringIntakeState state) {
        this.state = state;
        double vel = 0;
        switch (state) {
            case INIT:
                vel = INIT;
                break;
            case SHOOT:
                vel = SHOOT;
                break;
            case REVERSE:
                vel = REVERSE;
                break;
        }
        IntakeBack.setPower(vel);
    }
}
