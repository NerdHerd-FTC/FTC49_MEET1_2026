package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShootingConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.TurretSubsystem;

/**
 * Measure turretTicksPerDeg, then test the PID.
 * 1. Put the turret at a marked spot, INIT (encoder zeros). The motor floats; turn it by hand.
 * 2. Turn it exactly 90 deg CCW (seen from above). ticks / 90 = turretTicksPerDeg.
 *    If ticks went negative, set turretReversed = true and use the absolute value.
 * 3. Press START with the turret back at turretStartDeg: dpad left/up/right/down
 *    commands 90/0/-90/180 deg so you can tune turretKp/Ki/Kd live in Panels.
 */
@TeleOp(name = "TURRET TUNER", group = "Tuning")
public class TurretTuner extends LinearOpMode {
    @Override public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        while (opModeInInit()) {
            int ticks = motor.getCurrentPosition();
            telemetry.addData("Ticks", ticks);
            telemetry.addData("Ticks/deg if turned 90 CCW", "%.4f", Math.abs(ticks) / 90.0);
            telemetry.addData("Reverse?", ticks < 0 ? "YES: set turretReversed = true" : "no");
            telemetry.update();
        }
        if (isStopRequested()) return;

        // START: the turret must be back at turretStartDeg; zero again and run the real subsystem.
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, true);
        double target = Math.toRadians(ShootingConfig.turretStartDeg);
        try {
            while (opModeIsActive()) {
                if (gamepad1.dpad_left) target = Math.toRadians(90);
                if (gamepad1.dpad_up) target = 0;
                if (gamepad1.dpad_right) target = Math.toRadians(-90);
                if (gamepad1.dpad_down) target = Math.toRadians(180);
                turret.setAngle(target);
                telemetry.addData("Target / commanded / actual (deg)", "%.1f / %.1f / %.1f",
                        Math.toDegrees(target), Math.toDegrees(turret.getCommandedAngle()),
                        Math.toDegrees(turret.getAngle()));
                telemetry.addData("Ticks", turret.getTicks());
                telemetry.addData("On target", turret.onTarget());
                telemetry.update();
            }
        } finally {
            turret.stop();
        }
    }
}
