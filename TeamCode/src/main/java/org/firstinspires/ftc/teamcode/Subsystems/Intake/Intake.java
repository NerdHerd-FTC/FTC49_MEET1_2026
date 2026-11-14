package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake extends SubsystemBase {
    public DcMotor IntakeFront;

    // Static so Panels can edit them live.
    public static double INIT = 0;
    public static double FORWARD = 1;
    public static double BACKWARD = -1;

    public enum IntakeState{
        INIT,
        FORWARD,
        BACKWARD
    }

    public Intake (HardwareMap hardwareMap){
        this.IntakeFront = hardwareMap.get(DcMotor.class, "IntakeFront");

    }

    public void setState(IntakeState state) {
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
        IntakeFront.setPower(vel);
    }
}
