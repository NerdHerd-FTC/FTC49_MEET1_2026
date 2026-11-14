package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

/** Values carried between OpModes: Pedro field inches and heading in radians. */
public final class PoseStorage {
    private PoseStorage() { }

    // Autos and TeleOps update their own field every loop and on exit, including interrupted runs.
    // Null means no pose has been saved since the Robot Controller app started.
    public static Pose redPose;
    public static Pose bluePose;

    // True once any OpMode has zeroed the turret encoder at turretStartDeg. The hub keeps the
    // count between OpModes, so later OpModes must not zero it again wherever the turret is.
    public static boolean turretZeroed;
}
