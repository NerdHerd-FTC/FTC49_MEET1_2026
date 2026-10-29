package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ScoringSubsystem {
    public DcMotorEx Shooter;
    private double targetVelocity;
    private boolean pidfValid;

    private double appliedKp = Double.NaN;
    private double appliedKi = Double.NaN;
    private double appliedKd = Double.NaN;
    private double appliedKf = Double.NaN;

    public enum ShooterState{
        INIT,
        FORWARD,
        BACKWARD
    }

    public ScoringSubsystem (HardwareMap hardwareMap){
        this.Shooter = hardwareMap.get(DcMotorEx.class, "ShooterMotor");
        periodic();
    }

    public void periodic() {
        // Panels edits the fields; forward changed coefficients to the motor controller.
        double currentKp = ShootingConfig.kp;
        double currentKi = ShootingConfig.ki;
        double currentKd = ShootingConfig.kd;
        double currentKf = ShootingConfig.kf;
        pidfValid = Double.isFinite(currentKp) && Double.isFinite(currentKi)
                && Double.isFinite(currentKd) && Double.isFinite(currentKf);
        if (pidfValid && (currentKp != appliedKp || currentKi != appliedKi
                || currentKd != appliedKd || currentKf != appliedKf)) {
            Shooter.setVelocityPIDFCoefficients(currentKp, currentKi, currentKd, currentKf);
            appliedKp = currentKp;
            appliedKi = currentKi;
            appliedKd = currentKd;
            appliedKf = currentKf;
        }
    }

    public double getTargetVelocity() { return targetVelocity; }
    public boolean isPidfValid() { return pidfValid; }

    public void setVelocity(double velocity) {
        targetVelocity = Double.isFinite(velocity) ? velocity : 0;
        Shooter.setVelocity(targetVelocity);
    }

    public double getVelocity (){
        return Shooter.getVelocity();
    }

    public void setShooterState (ShooterState state) {
        double vel = 0;
        switch (state) {
            case INIT:
                vel = 0;
                break;
            case FORWARD:
                vel = ShootingConfig.autonomousVelocityTicksPerSecond;
                break;
            case BACKWARD:
                vel = -ShootingConfig.autonomousVelocityTicksPerSecond;
                break;
        }
        setVelocity(vel);
    }

}
