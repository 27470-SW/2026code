package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * Step 6b. Drives a DISTANCE x DISTANCE square with 90 degree turns, forever. Use it like
 * BackAndForth to tune TRANSLATIONAL_PID and HEADING_PID live in FTC Dashboard, with turns and
 * the field view (the robot should stay on the drawn path).
 */
@Config
@Autonomous(name = "RR 6b FollowerPIDTuner", group = "RR Tuning")
public class FollowerPIDTuner extends RRTuningOpMode
{
    public static double DISTANCE = 48; // in

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();

        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);
        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested())
        {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .build();
            drive.followTrajectorySequenceAsync(trajSeq);
            updateUntilIdle();
        }
    }
}
