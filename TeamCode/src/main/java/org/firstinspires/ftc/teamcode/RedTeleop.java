package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Alliance;

@TeleOp(name = "RED FIELD", group = "AA_Drive_Code")
public class RedTeleop extends AllianceTeleOp {
    @Override protected Alliance alliance() { return Alliance.RED; }
}
