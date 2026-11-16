package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Robot.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;

@TeleOp(name = "RED_FIELD", group = "AA_Drive_Code")
public class RedFieldTeleop extends CommandOpMode {
    @Override
    public void initialize(){
        MyRobot robot = new MyRobot(this, MyRobot.TeleopMode.FIELD, Alliance.RED);
    }
}
