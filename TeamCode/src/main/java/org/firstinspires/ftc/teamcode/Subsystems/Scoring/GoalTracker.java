package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.pedropathing.math.Pose;

/** Picks the nearer of an alliance's two goals each loop and calculates the shot for it. */
public final class GoalTracker {
    private final ShotTargeting targeting = new ShotTargeting();
    private boolean left;
    private boolean picked;

    public ShotTargeting.Solution update(Pose pose, ShotTargeting.Goal leftGoal, ShotTargeting.Goal rightGoal) {
        // First loop: plain closest goal. After that, the margin stops flip-flopping near the midline.
        left = ShotTargeting.pickLeft(pose, leftGoal, rightGoal, left,
                picked ? ShootingConfig.goalSwitchMarginIn : 0);
        picked = true;
        return targeting.calculate(pose, left ? leftGoal : rightGoal);
    }

    public boolean isLeft() { return left; }
}
