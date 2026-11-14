package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Alliance;

@Autonomous(name = "RED AUTO", group = "AA_Drive_Code", preselectTeleOp = "RED FIELD")
public class RedAuto extends AllianceAuto {
    @Override protected Alliance alliance() { return Alliance.RED; }
}
