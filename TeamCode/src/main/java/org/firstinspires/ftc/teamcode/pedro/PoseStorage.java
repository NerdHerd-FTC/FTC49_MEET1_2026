package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;

/** Actual auto ending poses for TeleOp: Pedro field inches and heading in radians. */
public final class PoseStorage {
    private PoseStorage() { }

    // RedAuto/BlueAuto update their own field every loop and on exit, including interrupted runs.
    // Null means no auto pose has been saved since the Robot Controller app started.
    public static Pose redPose;
    public static Pose bluePose;
}
