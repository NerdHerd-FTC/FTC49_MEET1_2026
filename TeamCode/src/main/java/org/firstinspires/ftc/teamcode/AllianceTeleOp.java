package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;

/** Shared TeleOp; RedTeleop and BlueTeleop only choose the alliance. */
public abstract class AllianceTeleOp extends LinearOpMode {
    protected abstract Alliance alliance();

    @Override public void runOpMode() throws InterruptedException {
        MyRobot robot = new MyRobot(this, alliance());
        boolean resetHeld = false;
        try {
            // Nothing is powered during INIT; the screen shows where the robot must be.
            while (opModeInInit()) robot.showInitTelemetry();
            while (opModeIsActive()) {
                if (gamepad1.y && !resetHeld) robot.resetPose();
                resetHeld = gamepad1.y;
                robot.update();
                robot.showTelemetry();
            }
        } finally {
            robot.stopAll();
        }
    }
}
