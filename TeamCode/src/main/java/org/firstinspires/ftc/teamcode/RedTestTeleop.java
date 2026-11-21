package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Robot.Alliance;

@TeleOp(name = "RED_TEST", group = "AA_Drive_Code")
public class RedTestTeleop extends TestTeleOp {
    @Override protected Alliance alliance() { return Alliance.RED; }
}
