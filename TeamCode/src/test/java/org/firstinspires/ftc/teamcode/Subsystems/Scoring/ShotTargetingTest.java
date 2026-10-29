package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.RedTeleop;
import org.firstinspires.ftc.teamcode.BlueTeleop;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;
import org.firstinspires.ftc.teamcode.pedro.PoseStorage;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class ShotTargetingTest {
    private final ShotTargeting targeting = new ShotTargeting();
    private static final String TABLE = "10:500,50:900,100:1200";

    @Before public void configure() {
        clearSavedPoses();
        ShootingConfig.redLeftXIn = ShootingConfig.redRightXIn = 59.25;
        ShootingConfig.blueLeftXIn = ShootingConfig.blueRightXIn = 84.75;
        ShootingConfig.redLeftYIn = ShootingConfig.blueRightYIn = 87.77;
        ShootingConfig.redRightYIn = ShootingConfig.blueLeftYIn = 56.23;
        ShootingConfig.shooterForwardIn = ShootingConfig.shooterLeftIn = ShootingConfig.shooterYawOffsetDeg = 0;
        ShootingConfig.redLeftShots = ShootingConfig.redRightShots =
                ShootingConfig.blueLeftShots = ShootingConfig.blueRightShots = TABLE;
        ShootingConfig.redLeftCalibrated = ShootingConfig.redRightCalibrated =
                ShootingConfig.blueLeftCalibrated = ShootingConfig.blueRightCalibrated = false;
        ShootingConfig.maxVelocityTicksPerSecond = 2000;
        ShootingConfig.headingToleranceDeg = 2.5;
        ShootingConfig.velocityToleranceTicksPerSecond = 50;
    }

    @After public void clearSavedPoses() {
        PoseStorage.redPose = null;
        PoseStorage.bluePose = null;
    }

    @Test public void autoPosesTransferToOnlyTheirMatchingTeleopUnchanged() {
        RedTeleop red = mock(RedTeleop.class, CALLS_REAL_METHODS);
        BlueTeleop blue = mock(BlueTeleop.class, CALLS_REAL_METHODS);
        Pose redEnd = new Pose(25,110,Math.toRadians(207));
        Pose blueEnd = new Pose(118,39,Math.toRadians(53));
        PoseStorage.redPose = redEnd;
        assertSame(redEnd, red.startingPose());
        assertEquals(ShootingConfig.blueStartXIn, blue.startingPose().x(), 0);
        assertEquals(ShootingConfig.blueStartYIn, blue.startingPose().y(), 0);
        assertEquals(Math.toRadians(ShootingConfig.blueStartHeadingDeg), blue.startingPose().heading(), 1e-9);
        PoseStorage.bluePose = blueEnd;
        assertSame(blueEnd, blue.startingPose());
        assertSame(redEnd, red.startingPose());
        PoseStorage.redPose = null;
        assertEquals(ShootingConfig.redStartXIn, red.startingPose().x(), 0);
        assertEquals(ShootingConfig.redStartYIn, red.startingPose().y(), 0);
        assertEquals(Math.toRadians(ShootingConfig.redStartHeadingDeg), red.startingPose().heading(), 1e-9);
        assertSame(blueEnd, blue.startingPose());
    }

    @Test public void teleopsSelectAllFourCellsFromTheirOperatorsPerspective() {
        RedTeleop red = mock(RedTeleop.class, CALLS_REAL_METHODS);
        BlueTeleop blue = mock(BlueTeleop.class, CALLS_REAL_METHODS);
        MyRobot robot = mock(MyRobot.class);
        red.updateRobot(robot,true);
        verify(robot).update(59.25,87.77,TABLE,false);
        red.updateRobot(robot,false);
        verify(robot).update(59.25,56.23,TABLE,false);
        blue.updateRobot(robot,true);
        verify(robot).update(84.75,56.23,TABLE,false);
        blue.updateRobot(robot,false);
        verify(robot).update(84.75,87.77,TABLE,false);
        ShotTargeting.Solution r = targeting.calculate(new Pose(0,87.77,0),59.25,87.77,TABLE,true);
        ShotTargeting.Solution b = targeting.calculate(new Pose(144,56.23,Math.PI),84.75,56.23,TABLE,true);
        assertEquals(r.distanceIn,b.distanceIn,1e-9);
        assertEquals(0,r.errorRadians,1e-9);
        assertEquals(0,b.errorRadians,1e-9);
    }

    @Test public void rotatesShooterOffsetWithRobotAndAppliesYawOffset() {
        ShootingConfig.shooterForwardIn = 10;
        ShootingConfig.shooterLeftIn = 2;
        ShootingConfig.shooterYawOffsetDeg = 10;
        ShotTargeting.Solution s = targeting.calculate(new Pose(20,30,Math.PI/2),18,70,TABLE,true);
        assertEquals(30,s.distanceIn,1e-9); // Release point is (18,40).
        assertEquals(80,Math.toDegrees(s.headingRadians),1e-9);
        assertEquals(-10,Math.toDegrees(s.errorRadians),1e-9);
    }

    @Test public void redCalibrationEditsDoNotChangeBlueSelection() {
        MyRobot robot = mock(MyRobot.class);
        BlueTeleop blue = mock(BlueTeleop.class,CALLS_REAL_METHODS);
        RedTeleop red = mock(RedTeleop.class,CALLS_REAL_METHODS);
        ShootingConfig.redLeftXIn = 40;
        ShootingConfig.redLeftYIn = 40;
        ShootingConfig.redLeftShots = "1:400,20:600";
        red.updateRobot(robot,true);
        verify(robot).update(40,40,"1:400,20:600",false);
        blue.updateRobot(robot,true);
        verify(robot).update(84.75,56.23,TABLE,false);
    }

    @Test public void positionChangesUpdateBothDistanceAndHeading() {
        ShotTargeting.Solution before = calculate(new Pose(19.25,56.23,0),TABLE,true);
        ShotTargeting.Solution pushed = calculate(new Pose(19.25,26.23,0),TABLE,true);
        assertEquals(40,before.distanceIn,1e-9);
        assertEquals(50,pushed.distanceIn,1e-9);
        assertEquals(800,before.velocityTicksPerSecond,1e-9);
        assertEquals(900,pushed.velocityTicksPerSecond,1e-9);
        assertEquals(Math.atan2(30,40),pushed.headingRadians,1e-9);
    }

    @Test public void usesNeighboringCalibrationPointsAndLiveEdits() {
        Pose pose = new Pose(59.25-75,56.23,0);
        assertEquals(1050,calculate(pose,TABLE,true).velocityTicksPerSecond,1e-9);
        assertEquals(1200,calculate(pose,"10:600,50:1000,100:1400",true).velocityTicksPerSecond,1e-9);
        assertEquals(1050,calculate(pose,TABLE,true).velocityTicksPerSecond,1e-9);
    }

    @Test public void invalidTablesBlockButCalibrationAndRangeAreOnlyWarnings() {
        Pose pose = new Pose(19.25,56.23,0);
        for (String bad : new String[]{"", "10:500", "10:500,10:600", "50:900,10:500",
                "10:500,50:NaN", "10:500,50:-1", "10:500,50:900,"}) {
            ShotTargeting.Solution invalid = calculate(pose,bad,true);
            assertEquals("Invalid shot table",invalid.warning);
            assertEquals(0,invalid.velocityTicksPerSecond,0);
            assertFalse(invalid.ready(0));
        }
        ShotTargeting.Solution far = calculate(new Pose(-20.75,56.23,0),"10:500,50:900",true);
        assertEquals(900,far.velocityTicksPerSecond,0);
        assertEquals("Outside calibrated range",far.warning);
        assertTrue(far.ready(900)); // Warning doesn't block a correctly aligned shot.
        ShotTargeting.Solution uncalibrated = calculate(pose,TABLE,false);
        assertEquals("Cell needs calibration",uncalibrated.warning);
        assertTrue(uncalibrated.ready(800));
        ShootingConfig.maxVelocityTicksPerSecond = 700;
        assertFalse(calculate(pose,TABLE,true).ready(0));
        assertFalse(calculate(new Pose(Double.NaN,0,0),TABLE,true).ready(0));
    }

    @Test public void headingWindowIsInclusiveAndWrapsAtZero() {
        for (double error : new double[]{-2.5,0,2.5}) assertTrue(solution(error).ready(950));
        assertFalse(solution(-2.501).ready(950));
        assertFalse(solution(2.501).ready(950));
        assertFalse(solution(0).ready(899));
        assertTrue(solution(0).ready(900));
        assertFalse(solution(0).ready(Double.NaN));
        assertEquals(2,Math.toDegrees(ShotTargeting.wrap(Math.toRadians(1-359))),1e-9);
        assertEquals(-2,Math.toDegrees(ShotTargeting.wrap(Math.toRadians(359-1))),1e-9);
    }

    private ShotTargeting.Solution calculate(Pose pose,String table,boolean calibrated) {
        return targeting.calculate(pose,59.25,56.23,table,calibrated);
    }

    private ShotTargeting.Solution solution(double error) {
        return new ShotTargeting.Solution(40,0,Math.toRadians(error),950,null);
    }
}
