package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.Alliance;

@Autonomous(name = "BLUE AUTO", group = "AA_Drive_Code", preselectTeleOp = "BLUE FIELD")
public class BlueAuto extends AllianceAuto {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
}
