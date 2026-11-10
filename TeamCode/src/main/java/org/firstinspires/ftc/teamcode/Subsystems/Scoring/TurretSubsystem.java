package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Encoder motor turret with its own PID. Angles are robot-relative radians, CCW positive,
 * 0 = robot forward (plus shooterYawOffsetDeg). Angle = turretStartDeg + encoder ticks / ticksPerDeg,
 * so the turret must sit at turretStartDeg whenever the encoder is zeroed.
 */
public class TurretSubsystem extends SubsystemBase {
    private final DcMotorEx motor;
    private double commandedRadians;
    private double integral;
    private double lastError = Double.NaN;
    private long lastTime;

    /** @param zeroEncoder true when the turret is at turretStartDeg (auto INIT, or no auto ran). */
    public TurretSubsystem(HardwareMap hardwareMap, boolean zeroEncoder) {
        motor = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (zeroEncoder) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Otherwise keep the count from auto: the hub holds encoder counts between OpModes.
        // Raw power for our PID; the encoder still counts. No power here, so no motion in INIT.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        commandedRadians = getAngle();
    }

    /** Call every loop: picks the reachable equivalent of the target, then runs one PID step. */
    public void setAngle(double radians) {
        double current = getAngle();
        double target = ShotTargeting.nearestTurretAngle(radians, current);
        if (!Double.isFinite(current) || !Double.isFinite(target)) {
            stop(); // Brake in place rather than swing on bad input or config.
            return;
        }
        commandedRadians = target;

        long now = System.nanoTime();
        double dt = lastTime == 0 ? 0 : (now - lastTime) / 1e9;
        lastTime = now;
        double error = Math.toDegrees(target - current);
        double maxPower = Math.max(0, Math.min(1, ShootingConfig.turretMaxPower));
        double kp = ShootingConfig.turretKp, ki = ShootingConfig.turretKi, kd = ShootingConfig.turretKd;
        if (!Double.isFinite(kp) || !Double.isFinite(ki) || !Double.isFinite(kd)) {
            stop();
            return;
        }
        double derivative = dt > 0 && Double.isFinite(lastError) ? (error - lastError) / dt : 0;
        lastError = error;
        if (dt > 0 && ki != 0) {
            integral += error * dt;
            // Anti-windup: the I term alone can never exceed max power.
            double limit = maxPower / Math.abs(ki);
            integral = Math.max(-limit, Math.min(limit, integral));
        }
        double power = kp * error + ki * integral + kd * derivative;
        motor.setPower(direction() * Math.max(-maxPower, Math.min(maxPower, power)));
    }

    public void center() { setAngle(0); }

    public void stop() {
        motor.setPower(0);
        integral = 0;
        lastError = Double.NaN;
        lastTime = 0;
    }

    public double getCommandedAngle() { return commandedRadians; }

    public int getTicks() { return motor.getCurrentPosition(); }

    public double getAngle() {
        double ticksPerDeg = ShootingConfig.turretTicksPerDeg;
        if (!Double.isFinite(ticksPerDeg) || ticksPerDeg <= 0) return Double.NaN;
        return Math.toRadians(ShootingConfig.turretStartDeg
                + direction() * motor.getCurrentPosition() / ticksPerDeg);
    }

    /** True when the measured angle is within tolerance of the commanded angle. */
    public boolean onTarget() {
        double tolerance = ShootingConfig.turretToleranceDeg;
        double error = getAngle() - commandedRadians;
        return Double.isFinite(error) && Double.isFinite(tolerance) && tolerance >= 0
                && Math.abs(error) <= Math.toRadians(tolerance) + 1e-12;
    }

    private static double direction() { return ShootingConfig.turretReversed ? -1 : 1; }
}
