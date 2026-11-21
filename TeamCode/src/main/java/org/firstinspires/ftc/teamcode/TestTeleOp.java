package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;


/** Shared TeleOp; RedTeleop and BlueTeleop only choose the alliance. */
public abstract class TestTeleOp extends LinearOpMode {
    protected abstract Alliance alliance();


    @Override public void runOpMode() throws InterruptedException {
        boolean resetHeld = false;
        try {
            // Nothing is powered during INIT; the screen shows where the robot must be.
            waitForStart();
            MyRobot robot = new MyRobot(this, MyRobot.TeleopMode.TEST, alliance());
            while (opModeIsActive()) {
                if (gamepad1.y && !resetHeld) robot.resetPose();
                resetHeld = gamepad1.y;
                robot.update();
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
