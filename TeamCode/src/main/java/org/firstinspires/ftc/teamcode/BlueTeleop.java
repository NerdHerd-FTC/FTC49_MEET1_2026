package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Alliance;

@TeleOp(name = "BLUE FIELD", group = "AA_Drive_Code")
public class BlueTeleop extends AllianceTeleOp {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
}
