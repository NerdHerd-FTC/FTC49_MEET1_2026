package org.firstinspires.ftc.teamcode.Subsystems.Scoring;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.reflect.Field;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot.MyRobot;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoringIntakeSubsystem.ScoringIntakeState;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class ShootingControlTest {
    private DriveSubsystem drive;
    private ScoringSubsystem shooter;
    private ScoringIntakeSubsystem feeder;
    private Intake intake;
    private LinearOpMode opMode;
    private MyRobot robot;

    @Before public void setup() throws Exception {
        ShootingConfig.headingToleranceDeg = 2.5;
        ShootingConfig.velocityToleranceTicksPerSecond = 50;
        ShootingConfig.triggerThreshold = 0.2;
        ShootingConfig.maxVelocityTicksPerSecond = 2000;
        ShootingConfig.shooterForwardIn = ShootingConfig.shooterLeftIn = ShootingConfig.shooterYawOffsetDeg = 0;
        ShootingConfig.kp = 150; ShootingConfig.ki = 0; ShootingConfig.kd = 0; ShootingConfig.kf = 0;
        drive = mock(DriveSubsystem.class);
        shooter = mock(ScoringSubsystem.class);
        feeder = mock(ScoringIntakeSubsystem.class);
        intake = mock(Intake.class);
        opMode = mock(LinearOpMode.class);
        opMode.gamepad1 = new Gamepad();
        opMode.gamepad2 = new Gamepad();
        robot = mock(MyRobot.class,CALLS_REAL_METHODS);
        for (Object[] entry : new Object[][]{{"opMode",opMode},{"drive",drive},{"shooter",shooter},
                {"feeder",feeder},{"intake",intake},{"targeting",new ShotTargeting()}}) {
            Field field = MyRobot.class.getDeclaredField((String)entry[0]);
            field.setAccessible(true);
            field.set(robot,entry[1]);
        }
        when(drive.getPose()).thenReturn(new Pose(19.25,56.23,0));
        when(shooter.getVelocity()).thenReturn(800.0);
    }

    @Test public void automaticShotFeedsImmediatelyWithinToleranceAndPausesOutsideIt() {
        opMode.gamepad2.right_trigger = 1;
        for (double heading : new double[]{-2.5,0,2.5}) {
            when(drive.getPose()).thenReturn(new Pose(19.25,56.23,Math.toRadians(heading)));
            clearInvocations(feeder);
            update();
            verify(feeder).setScoringIntakeState(ScoringIntakeState.SHOOT);
        }
        verify(drive,times(1)).beginAim();
        when(drive.getPose()).thenReturn(new Pose(19.25,56.23,Math.toRadians(2.501)));
        clearInvocations(feeder);
        update();
        verify(feeder).setScoringIntakeState(ScoringIntakeState.INIT);
        when(drive.getPose()).thenReturn(new Pose(19.25,56.23,0));
        when(shooter.getVelocity()).thenReturn(749.0);
        clearInvocations(feeder);
        update();
        verify(feeder).setScoringIntakeState(ScoringIntakeState.INIT);
    }

    @Test public void releaseReturnsManualDriveAndReverseWinsOverBothTriggers() {
        opMode.gamepad2.right_trigger = 1;
        update();
        opMode.gamepad2.right_trigger = 0;
        opMode.gamepad1.left_stick_x = 0.25f;
        opMode.gamepad1.left_stick_y = -0.5f;
        clearInvocations(drive,feeder);
        update();
        verify(drive).setDrivePower(0.25,0.5,0);
        verify(drive,never()).driveToHeading(anyDouble());
        verify(feeder).setScoringIntakeState(ScoringIntakeState.INIT);
        opMode.gamepad1.x = true;
        opMode.gamepad2.left_trigger = opMode.gamepad2.right_trigger = 1;
        clearInvocations(drive,feeder);
        update();
        verify(drive).setDrivePower(0.25,0.5,0);
        verify(drive,never()).driveToHeading(anyDouble());
        verify(feeder).setScoringIntakeState(ScoringIntakeState.REVERSE);
        verify(intake).setState(Intake.IntakeState.FORWARD);
    }

    @Test public void leftTriggerAloneFeedsAndWinsOverRightWhileKeepingSlowManualDrive() {
        opMode.gamepad1.left_bumper = true;
        opMode.gamepad1.left_stick_x = 0.25f;
        opMode.gamepad1.left_stick_y = -0.5f;
        opMode.gamepad2.left_trigger = 1;
        update();
        verify(feeder).setScoringIntakeState(ScoringIntakeState.SHOOT);
        verify(drive).setDrivePower(0.125,0.25,0);
        opMode.gamepad2.right_trigger = 1;
        update();
        verify(drive,never()).driveToHeading(anyDouble());
        opMode.gamepad2.left_trigger = 0;
        update();
        verify(drive).beginAim();
        verify(drive).driveToHeading(0);
    }

    @Test public void manualDrivingAndPushingStillUpdateShooterSpeedEveryLoop() {
        update();
        verify(shooter).setVelocity(800);
        when(drive.getPose()).thenReturn(new Pose(19.25,26.23,0));
        update();
        verify(shooter).setVelocity(900);
        verify(drive,times(2)).updatePose();
        verify(drive,never()).driveToHeading(anyDouble());
        opMode.gamepad2.right_trigger = 1;
        update();
        verify(drive).driveToHeading(doubleThat(h -> Math.abs(h - Math.atan2(30,40)) < 1e-9));
        verify(shooter,times(3)).periodic();
        verify(shooter,times(2)).setVelocity(900);
    }

    @Test public void panelsPidfEditsApplyOnceAndSpeedIsSetDirectly() {
        DcMotorEx motor = mock(DcMotorEx.class);
        ScoringSubsystem actualShooter = shooterWith(motor);
        actualShooter.setVelocity(950);
        verify(motor).setVelocity(950);
        ShootingConfig.kp = 170; ShootingConfig.ki = 1; ShootingConfig.kd = 2; ShootingConfig.kf = 3;
        actualShooter.periodic();
        actualShooter.setVelocity(1000);
        actualShooter.periodic();
        verify(motor).setVelocity(1000);
        verify(motor,times(1)).setVelocityPIDFCoefficients(170,1,2,3);
    }

    @Test public void invalidPidfEditsKeepLastGainsAndAllowDirectSpeedUpdates() {
        DcMotorEx motor = mock(DcMotorEx.class);
        ScoringSubsystem actualShooter = shooterWith(motor);
        clearInvocations(motor);
        for (double[] gains : new double[][]{{Double.NaN,0,0,0},{150,Double.POSITIVE_INFINITY,0,0},
                {150,0,Double.NEGATIVE_INFINITY,0},{150,0,0,Double.NaN}}) {
            ShootingConfig.kp=gains[0]; ShootingConfig.ki=gains[1];
            ShootingConfig.kd=gains[2]; ShootingConfig.kf=gains[3];
            actualShooter.periodic();
            actualShooter.setVelocity(950);
            assertFalse(actualShooter.isPidfValid());
        }
        verify(motor,never()).setVelocityPIDFCoefficients(anyDouble(),anyDouble(),anyDouble(),anyDouble());
        verify(motor,times(4)).setVelocity(950);
        ShootingConfig.kp=170; ShootingConfig.ki=1; ShootingConfig.kd=2; ShootingConfig.kf=3;
        actualShooter.periodic();
        assertTrue(actualShooter.isPidfValid());
        verify(motor).setVelocityPIDFCoefficients(170,1,2,3);
    }

    @Test public void poseResetRestartsAimAndUsesCorrectedDistanceAndHeading() {
        opMode.gamepad2.right_trigger = 1;
        update();
        Pose corner = new Pose(19.25,26.23,0);
        robot.resetPose(corner);
        verify(drive).setPose(corner);
        when(drive.getPose()).thenReturn(corner);
        clearInvocations(feeder);
        update();
        verify(drive,times(2)).beginAim();
        verify(shooter).setVelocity(900);
        verify(drive).driveToHeading(doubleThat(h -> Math.abs(h - Math.atan2(30,40)) < 1e-9));
        verify(feeder).setScoringIntakeState(ScoringIntakeState.INIT);
    }

    @Test public void stoppingZerosEverySubsystem() {
        robot.stopAll();
        verify(drive).stop();
        verify(shooter).setVelocity(0);
        verify(feeder).setScoringIntakeState(ScoringIntakeState.INIT);
        verify(intake).setState(Intake.IntakeState.INIT);
    }

    private void update() {
        robot.update(59.25,56.23,"10:500,50:900,100:1200",false);
    }

    private ScoringSubsystem shooterWith(DcMotorEx motor) {
        HardwareMap map = mock(HardwareMap.class);
        when(map.get(DcMotorEx.class,"ShooterMotor")).thenReturn(motor);
        return new ScoringSubsystem(map);
    }
}
