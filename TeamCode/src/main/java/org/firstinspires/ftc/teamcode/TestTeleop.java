package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;

@TeleOp(name = "FIELD", group = "AA_Drive_Code")
public class TestTeleop extends CommandOpMode {
    @Override
    public void initialize(){
        MyRobot robot = new MyRobot(this, MyRobot.TeleopMode.FIELD);
    }
}
