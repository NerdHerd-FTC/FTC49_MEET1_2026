package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShootingConfig;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;

@TeleOp(name = "RED FIELD", group = "AA_Drive_Code")
public class RedTeleop extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        MyRobot robot = new MyRobot(this, startingPose());
        boolean left = false;
        boolean resetHeld = false;
        try {
            telemetry.addData("TeleOp", "RED FIELD");
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) {
                if (gamepad1.y && !resetHeld) {
                    robot.resetPose(new Pose(ShootingConfig.redResetXIn, ShootingConfig.redResetYIn,
                            Math.toRadians(ShootingConfig.redResetHeadingDeg)));
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
        return PoseStorage.redPose != null ? PoseStorage.redPose
                : new Pose(ShootingConfig.redStartXIn, ShootingConfig.redStartYIn,
                        Math.toRadians(ShootingConfig.redStartHeadingDeg));
    }

    // LEFT/RIGHT are the operator's view from the red station.
    public void updateRobot(MyRobot robot, boolean left) {
        robot.update(left ? ShootingConfig.redLeftXIn : ShootingConfig.redRightXIn,
                left ? ShootingConfig.redLeftYIn : ShootingConfig.redRightYIn,
                left ? ShootingConfig.redLeftShots : ShootingConfig.redRightShots,
                left ? ShootingConfig.redLeftCalibrated : ShootingConfig.redRightCalibrated);
        robot.showTelemetry("RED", left);
    }
}
