package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Robot.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;

@TeleOp(name = "BLUE_FIELD", group = "AA_Drive_Code")
public class BlueFieldTeleop extends AllianceTeleOp {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
}
