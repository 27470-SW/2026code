package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Step 7. Final check: spline out to (30, 30) and back. Watch the field view in FTC Dashboard. */
@Autonomous(name = "RR 7 SplineTest", group = "RR Tuning")
public class SplineTest extends RRTuningOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();

        waitForStart();
        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        drive.followTrajectoryAsync(traj);
        updateUntilIdle();
        if (isStopRequested()) return;

        sleep(2000);

        drive.followTrajectoryAsync(drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build());
        updateUntilIdle();

        StraightTest.reportEndPose(this, 0, 0);
    }
}
