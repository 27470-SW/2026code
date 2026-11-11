package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Sanity check: strafe right DISTANCE inches and report the end pose. */
@Config
@Autonomous(name = "RR 5b StrafeTest", group = "RR Tuning")
public class StrafeTest extends RRTuningOpMode
{
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d()).strafeRight(DISTANCE).build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectoryAsync(trajectory);
        updateUntilIdle();

        StraightTest.reportEndPose(this, 0, -DISTANCE);
    }
}
