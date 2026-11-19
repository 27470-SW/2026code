package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Step 6a. Translational/heading feedback tuning (RR 1.0 equivalent: ManualFeedbackTuner).
 *
 * Drives forward and back DISTANCE inches forever. In FTC Dashboard, graph xError, yError and
 * "headingError (deg)" and edit RobotConstants.TRANSLATIONAL_PID and HEADING_PID live; changes
 * take effect immediately. Start with kI = kD = 0 and raise kP until errors stay small without
 * oscillation, then add a little kD to damp overshoot. (TRANSLATIONAL_PID is used for both the
 * axial and lateral directions.)
 *
 * Tune feedforward first: feedback should only clean up small errors.
 */
@Config
@Autonomous(name = "RR 6a BackAndForth", group = "RR Tuning")
public class BackAndForth extends RRTuningOpMode
{
    public static double DISTANCE = 50; // in

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();
        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .back(DISTANCE)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested())
        {
            drive.followTrajectoryAsync(trajectoryForward);
            updateUntilIdle();
            drive.followTrajectoryAsync(trajectoryBackward);
            updateUntilIdle();
        }
    }
}
