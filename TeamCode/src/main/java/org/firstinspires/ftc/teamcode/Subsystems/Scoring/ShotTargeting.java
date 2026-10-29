package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.pedropathing.math.Pose;

/** Geometry and calibration only: no hardware, motor commands, or odometry resets. */
public final class ShotTargeting {
    private String cachedTable;
    private double[] distances = new double[0];
    private double[] velocities = new double[0];

    public Solution calculate(Pose pose, double goalX, double goalY, String shots, boolean calibrated) {
        double h = pose.heading();
        double shooterX = pose.x() + ShootingConfig.shooterForwardIn * Math.cos(h)
                - ShootingConfig.shooterLeftIn * Math.sin(h);
        double shooterY = pose.y() + ShootingConfig.shooterForwardIn * Math.sin(h)
                + ShootingConfig.shooterLeftIn * Math.cos(h);
        double distance = Math.hypot(goalX - shooterX, goalY - shooterY);
        double heading = Math.atan2(goalY - shooterY, goalX - shooterX)
                - Math.toRadians(ShootingConfig.shooterYawOffsetDeg);
        double error = wrap(heading - h);
        parseIfChanged(shots);
        if (!Double.isFinite(distance) || distance < 0.001 || !Double.isFinite(error)) {
            return new Solution(distance, heading, error, 0, "Invalid pose/target");
        }
        if (distances.length < 2) {
            return new Solution(distance, heading, error, 0, "Invalid shot table");
        }
        double velocity = interpolate(distance, distances, velocities);
        double limit = ShootingConfig.maxVelocityTicksPerSecond;
        if (!Double.isFinite(limit) || limit <= 0 || velocity > limit) {
            return new Solution(distance, heading, error, 0, "Velocity limit / config");
        }
        String warning = !calibrated ? "Cell needs calibration"
                : distance < distances[0] || distance > distances[distances.length - 1]
                ? "Outside calibrated range" : null;
        return new Solution(distance, heading, error, velocity, warning);
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

    public static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    public static final class Solution {
        public final double distanceIn, headingRadians, errorRadians, velocityTicksPerSecond;
        public final String warning;

        public Solution(double distance, double heading, double error, double velocity, String warning) {
            distanceIn = distance;
            headingRadians = heading;
            errorRadians = error;
            velocityTicksPerSecond = velocity;
            this.warning = warning;
        }

        /** No timer, motion threshold, or exact-zero-angle requirement. */
        public boolean ready(double actualVelocity) {
            double headingTolerance = ShootingConfig.headingToleranceDeg;
            double speedTolerance = ShootingConfig.velocityToleranceTicksPerSecond;
            return velocityTicksPerSecond > 0 && Double.isFinite(errorRadians)
                    && Double.isFinite(headingTolerance) && headingTolerance >= 0
                    && Double.isFinite(speedTolerance) && speedTolerance >= 0
                    && Math.abs(errorRadians) <= Math.toRadians(headingTolerance) + 1e-12
                    && Double.isFinite(actualVelocity)
                    && Math.abs(actualVelocity - velocityTicksPerSecond) <= speedTolerance;
        }
    }
}
