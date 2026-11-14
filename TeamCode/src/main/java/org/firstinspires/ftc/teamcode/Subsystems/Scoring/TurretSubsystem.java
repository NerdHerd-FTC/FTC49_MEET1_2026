package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

/**
 * Encoder motor turret with its own PID. Angles are robot-relative radians, CCW positive,
 * 0 = robot forward (plus shooterYawOffsetDeg). Angle = turretStartDeg + encoder ticks / ticksPerDeg,
 * so the turret must sit at turretStartDeg whenever the encoder is zeroed.
 */
public class TurretSubsystem extends SubsystemBase {
    private final DcMotorEx motor;
    // Read once at INIT: a live edit would make the same encoder count mean a different angle.
    private final double ticksPerDeg;
    private final double startDeg;
    private final double direction;
    private double commandedRadians;
    private double integral;
    private double lastAngle = Double.NaN;
    private long lastTime;

    /** @param zeroEncoder true when the turret is at turretStartDeg and no OpMode has zeroed it yet. */
    public TurretSubsystem(HardwareMap hardwareMap, boolean zeroEncoder) {
        motor = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        ticksPerDeg = ShootingConfig.turretTicksPerDeg;
        startDeg = ShootingConfig.turretStartDeg;
        direction = ShootingConfig.turretReversed ? -1 : 1;
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (zeroEncoder) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            PoseStorage.turretZeroed = true;
        }
        // Otherwise keep the count: the hub holds encoder counts between OpModes.
        // Raw power for our PID; the encoder still counts. No power here, so no motion in INIT.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
        commandedRadians = getAngle();
    }

    /** Call every loop: picks the reachable equivalent of the target, then runs one PID step. */
    public void setAngle(double radians) {
        double current = getAngle();
        double target = ShotTargeting.nearestTurretAngle(radians, current);
        double kp = ShootingConfig.turretKp, ki = ShootingConfig.turretKi, kd = ShootingConfig.turretKd;
        if (!Double.isFinite(current) || !Double.isFinite(target)
                || !Double.isFinite(kp) || !Double.isFinite(ki) || !Double.isFinite(kd)) {
            stop(); // Brake in place rather than swing on bad input or config.
            return;
        }
        commandedRadians = target;

        long now = System.nanoTime();
        double dt = lastTime == 0 ? 0 : (now - lastTime) / 1e9;
        lastTime = now;
        double error = Math.toDegrees(target - current);
        double maxPower = Math.max(0, Math.min(1, ShootingConfig.turretMaxPower));
        // D on the measured angle, not the error: a target jump (goal switch, wrap) causes no kick.
        double rate = dt > 0 && Double.isFinite(lastAngle) ? Math.toDegrees(current - lastAngle) / dt : 0;
        lastAngle = current;
        if (dt > 0 && ki != 0) {
            integral += error * dt;
            // Anti-windup: the I term alone can never exceed max power.
            double limit = maxPower / Math.abs(ki);
            integral = Math.max(-limit, Math.min(limit, integral));
        }
        double power = kp * error + ki * integral - kd * rate;
        // Past a travel limit (overshoot, bad zero): only allow power back toward the range.
        if (current > Math.toRadians(ShootingConfig.turretMaxDeg)) power = Math.min(power, 0);
        if (current < Math.toRadians(ShootingConfig.turretMinDeg)) power = Math.max(power, 0);
        motor.setPower(direction * Math.max(-maxPower, Math.min(maxPower, power)));
    }

    public void center() { setAngle(0); }

    public void stop() {
        motor.setPower(0);
        integral = 0;
        lastAngle = Double.NaN;
        lastTime = 0;
    }

    public double getCommandedAngle() { return commandedRadians; }

    public int getTicks() { return motor.getCurrentPosition(); }

    public double getAngle() {
        if (!Double.isFinite(ticksPerDeg) || ticksPerDeg <= 0 || !Double.isFinite(startDeg)) return Double.NaN;
        return Math.toRadians(startDeg + direction * motor.getCurrentPosition() / ticksPerDeg);
    }

    /** True when the measured angle is within tolerance of the commanded angle. */
    public boolean onTarget() {
        double tolerance = ShootingConfig.turretToleranceDeg;
        double error = getAngle() - commandedRadians;
        return Double.isFinite(error) && Double.isFinite(tolerance) && tolerance >= 0
                && Math.abs(error) <= Math.toRadians(tolerance) + 1e-12;
    }
}
