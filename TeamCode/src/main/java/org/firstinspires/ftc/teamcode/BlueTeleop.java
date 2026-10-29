package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShootingConfig;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

@TeleOp(name = "BLUE FIELD", group = "AA_Drive_Code")
public class BlueTeleop extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        MyRobot robot = new MyRobot(this, startingPose());
        boolean left = false;
        boolean resetHeld = false;
        try {
            telemetry.addData("TeleOp", "BLUE FIELD");
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) {
                if (gamepad1.y && !resetHeld) {
                    robot.resetPose(new Pose(ShootingConfig.blueResetXIn, ShootingConfig.blueResetYIn,
                            Math.toRadians(ShootingConfig.blueResetHeadingDeg)));
                }
                resetHeld = gamepad1.y;
                if (gamepad2.left_bumper && !gamepad2.right_bumper) left = true;
                if (gamepad2.right_bumper && !gamepad2.left_bumper) left = false;
                updateRobot(robot, left);
            }
        } finally {
            robot.stopAll();
        }
    }

    public Pose startingPose() {
        return PoseStorage.bluePose != null ? PoseStorage.bluePose
                : new Pose(ShootingConfig.blueStartXIn, ShootingConfig.blueStartYIn,
                        Math.toRadians(ShootingConfig.blueStartHeadingDeg));
    }

    // LEFT/RIGHT are the operator's view from the blue station.
    public void updateRobot(MyRobot robot, boolean left) {
        robot.update(left ? ShootingConfig.blueLeftXIn : ShootingConfig.blueRightXIn,
                left ? ShootingConfig.blueLeftYIn : ShootingConfig.blueRightYIn,
                left ? ShootingConfig.blueLeftShots : ShootingConfig.blueRightShots,
                left ? ShootingConfig.blueLeftCalibrated : ShootingConfig.blueRightCalibrated);
        robot.showTelemetry("BLUE", left);
    }
}
