package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.pedropathing.math.Pose;

/** Geometry and calibration only: no hardware, motor commands, or odometry resets. */
public final class ShotTargeting {
    private String cachedTable;
    private double[] distances = new double[0];
    private double[] velocities = new double[0];

    public Solution calculate(Pose pose, Goal goal) {
        return calculate(pose, goal.x, goal.y, goal.shots, goal.calibrated);
    }

    public Solution calculate(Pose pose, double goalX, double goalY, String shots, boolean calibrated) {
        double h = pose.heading();
        double shooterX = pivotX(pose);
        double shooterY = pivotY(pose);
        double distance = Math.hypot(goalX - shooterX, goalY - shooterY);
        // Robot-relative turret angle; the drivetrain is never turned to aim.
        double turret = wrap(Math.atan2(goalY - shooterY, goalX - shooterX) - h
                - Math.toRadians(ShootingConfig.shooterYawOffsetDeg));
        boolean reachable = Double.isFinite(equivalentInRange(turret, 0));
        parseIfChanged(shots);
        if (!Double.isFinite(distance) || distance < 0.001 || !Double.isFinite(turret)) {
            return new Solution(distance, turret, false, 0, "Invalid pose/target");
        }
        if (distances.length < 2) {
            return new Solution(distance, turret, reachable, 0, "Invalid shot table");
        }
        double velocity = interpolate(distance, distances, velocities);
        // Every problem that applies, not just the first.
        StringBuilder warning = new StringBuilder();
        if (!reachable) warning.append("Target outside turret travel; turn robot. ");
        if (distance < distances[0] || distance > distances[distances.length - 1]) {
            warning.append("Outside calibrated range. ");
        }
        if (!calibrated) warning.append("Cell needs calibration.");
        String text = warning.toString().trim();
        return new Solution(distance, turret, reachable, velocity, text.isEmpty() ? null : text);
    }

    /**
     * Nearest-goal choice with hysteresis: switch only when the other goal is closer to the turret
     * pivot by at least marginIn, so the turret does not flip back and forth near the midline.
     */
    public static boolean pickLeft(Pose pose, Goal left, Goal right, boolean currentlyLeft,
            double marginIn) {
        double x = pivotX(pose), y = pivotY(pose);
        double toLeft = Math.hypot(left.x - x, left.y - y);
        double toRight = Math.hypot(right.x - x, right.y - y);
        if (!Double.isFinite(toLeft) || !Double.isFinite(toRight)) return currentlyLeft;
        double margin = Double.isFinite(marginIn) ? Math.max(0, marginIn) : 0;
        return currentlyLeft ? toLeft - toRight < margin : toRight - toLeft >= margin;
    }

    /** Turret pivot position on the field: robot pose plus the pivot offset rotated by heading. */
    private static double pivotX(Pose pose) {
        double h = pose.heading();
        return pose.x() + ShootingConfig.shooterForwardIn * Math.cos(h)
                - ShootingConfig.shooterLeftIn * Math.sin(h);
    }

    private static double pivotY(Pose pose) {
        double h = pose.heading();
        return pose.y() + ShootingConfig.shooterForwardIn * Math.sin(h)
                + ShootingConfig.shooterLeftIn * Math.cos(h);
    }

    private void parseIfChanged(String table) {
        if (table != null && table.equals(cachedTable)) return;
        cachedTable = table;
        distances = new double[0];
        velocities = new double[0];
        if (table == null) return;
        try {
            String[] rows = table.split(",", -1);
            if (rows.length < 2) return;
            double[] d = new double[rows.length];
            double[] v = new double[rows.length];
            for (int i = 0; i < rows.length; i++) {
                String[] pair = rows[i].trim().split(":", -1);
                if (pair.length != 2) return;
                d[i] = Double.parseDouble(pair[0]);
                v[i] = Double.parseDouble(pair[1]);
                if (!Double.isFinite(d[i]) || !Double.isFinite(v[i]) || d[i] < 0 || v[i] <= 0
                        || (i > 0 && d[i] <= d[i - 1])) return;
            }
            distances = d;
            velocities = v;
        } catch (NumberFormatException ignored) {
            // A partially edited Panels string must not keep using an old valid table.
        }
    }

    private static double interpolate(double distance, double[] d, double[] v) {
        if (distance <= d[0]) return v[0];
        for (int i = 1; i < d.length; i++) {
            if (distance <= d[i]) {
                double t = (distance - d[i - 1]) / (d[i] - d[i - 1]);
                return v[i - 1] + t * (v[i] - v[i - 1]);
            }
        }
        return v[v.length - 1];
    }

    /**
     * Wrap-around: of every equivalent angle (desired + k*360) inside the turret limits, the one
     * nearest the current angle, so the turret takes the shortest legal path. If none fits,
     * the limit closest to the desired direction. NaN on invalid input or limits (the turret
     * then brakes), including limits spanning more than 720 degrees.
     */
    public static double nearestTurretAngle(double desired, double current) {
        double min = Math.toRadians(ShootingConfig.turretMinDeg);
        double max = Math.toRadians(ShootingConfig.turretMaxDeg);
        if (!Double.isFinite(desired) || !Double.isFinite(min) || !Double.isFinite(max) || min > max
                || max - min > 4 * Math.PI) {
            return Double.NaN;
        }
        double best = equivalentInRange(desired, Double.isFinite(current) ? current : 0);
        if (Double.isFinite(best)) return best;
        return Math.abs(wrap(desired - min)) <= Math.abs(wrap(desired - max)) ? min : max;
    }

    private static double equivalentInRange(double desired, double reference) {
        double min = Math.toRadians(ShootingConfig.turretMinDeg);
        double max = Math.toRadians(ShootingConfig.turretMaxDeg);
        if (!Double.isFinite(desired) || !Double.isFinite(reference) || !Double.isFinite(min)
                || !Double.isFinite(max) || min > max || max - min > 4 * Math.PI) {
            return Double.NaN;
        }
        double best = Double.NaN;
        double base = desired + 2 * Math.PI * Math.round((reference - desired) / (2 * Math.PI));
        for (int k = -2; k <= 2; k++) {
            double candidate = base + 2 * Math.PI * k;
            if (candidate >= min - 1e-12 && candidate <= max + 1e-12
                    && (!Double.isFinite(best) || Math.abs(candidate - reference) < Math.abs(best - reference))) {
                best = candidate;
            }
        }
        return best;
    }

    public static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    /** One scoring opening: field position plus its own shot table. */
    public static final class Goal {
        public final double x, y;
        public final String shots;
        public final boolean calibrated;

        public Goal(double x, double y, String shots, boolean calibrated) {
            this.x = x;
            this.y = y;
            this.shots = shots;
            this.calibrated = calibrated;
        }
    }

    public static final class Solution {
        public final double distanceIn, turretRadians, velocityTicksPerSecond;
        public final boolean turretReachable;
        public final String warning;

        public Solution(double distance, double turret, boolean reachable, double velocity,
                String warning) {
            distanceIn = distance;
            turretRadians = turret;
            turretReachable = reachable;
            velocityTicksPerSecond = velocity;
            this.warning = warning;
        }
    }
}
