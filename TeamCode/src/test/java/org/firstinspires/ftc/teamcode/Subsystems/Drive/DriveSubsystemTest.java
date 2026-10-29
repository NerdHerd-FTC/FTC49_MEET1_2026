package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import java.lang.reflect.Field;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.ShootingConfig;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class DriveSubsystemTest {
    @Test public void fieldCentricSticksKeepPedroAxesAtEveryRobotHeading() throws Exception {
        Localizer localizer = mock(Localizer.class);
        Drivetrain motors = mock(Drivetrain.class);
        DriveSubsystem drive = mock(DriveSubsystem.class, CALLS_REAL_METHODS);
        put(drive,"follower",new Follower(localizer,motors,mock(Foresight.class)));
        for (double heading : new double[]{0, Math.PI/2, Math.PI, 3*Math.PI/2, Math.toRadians(207)}) {
                when(localizer.pose()).thenReturn(new Pose(40,50,heading));
                clearInvocations(motors);
                drive.setDrivePower(0,1,0);
                ArgumentCaptor<DrivePowers> forward = ArgumentCaptor.forClass(DrivePowers.class);
                verify(motors).drive(forward.capture(),eq(true));
                assertWorldTranslation(forward.getValue(),heading,1,0);

                clearInvocations(motors);
                drive.setDrivePower(1,0,0.3);
                ArgumentCaptor<DrivePowers> right = ArgumentCaptor.forClass(DrivePowers.class);
                verify(motors).drive(right.capture(),eq(true));
                assertWorldTranslation(right.getValue(),heading,0,-1);
                assertEquals(-0.3,right.getValue().turn(),1e-9);
        }
    }

    private void assertWorldTranslation(DrivePowers powers,double heading,double x,double y) {
        assertEquals(x,powers.forward()*Math.cos(heading)-powers.strafe()*Math.sin(heading),1e-9);
        assertEquals(y,powers.forward()*Math.sin(heading)+powers.strafe()*Math.cos(heading),1e-9);
    }

    @Test public void realPedroHoldTurnsButDoesNotFightTranslationWhilePushed() throws Exception {
        Pose pose = new Pose(40,50,0.4);
        Localizer localizer = mock(Localizer.class);
        when(localizer.pose()).thenReturn(pose);
        when(localizer.state()).thenReturn(MotionState.ofTwist(pose, new Twist(20,-12,0.3)));
        Drivetrain motors = mock(Drivetrain.class);
        when(motors.maxScaling(any(),any())).thenReturn(1.0);
        Foresight controller = new Foresight(Constants.foresightConfig);
        DriveSubsystem drive = mock(DriveSubsystem.class, CALLS_REAL_METHODS);
        put(drive,"follower",new Follower(localizer,motors,controller));
        put(drive,"holdController",controller);
        put(drive,"deltaTime",0.02);
        ShootingConfig.aimHeadingScale = 0.35;
        double oldTranslation = Constants.foresightConfig.holdPointTranslationalScaling.get();
        double oldHeading = Constants.foresightConfig.holdPointHeadingScaling.get();
        drive.driveToHeading(1.0);
        ArgumentCaptor<DrivePowers> output = ArgumentCaptor.forClass(DrivePowers.class);
        verify(motors).drive(output.capture(),eq(false));
        assertEquals(0,output.getValue().forward(),1e-9);
        assertEquals(0,output.getValue().strafe(),1e-9);
        assertTrue(Double.isFinite(output.getValue().turn()));
        assertTrue(output.getValue().turn() > 0);
        assertEquals(oldTranslation,Constants.foresightConfig.holdPointTranslationalScaling.get(),0);
        assertEquals(oldHeading,Constants.foresightConfig.holdPointHeadingScaling.get(),0);
        verify(localizer,never()).update(); // Output calculation doesn't resample pose.
        drive.stop();
        drive.setDrivePower(0,0,0);
        verify(motors).stop();
        verify(motors).drive(any(),eq(true));
    }

    private void put(Object object,String name,Object value) throws Exception {
        Field field=DriveSubsystem.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(object,value);
    }
}
