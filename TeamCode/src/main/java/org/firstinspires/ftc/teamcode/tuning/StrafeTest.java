package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

import java.util.Locale;

/**
 * Strafe right DISTANCE inches. Two uses (see tuning/README.md):
 *
 *  FEEDBACK = false (feedforward only) -> tunes LATERAL_MULTIPLIER (run after kV is right).
 *    Mecanum wheels slip sideways, so strafing needs extra power; LATERAL_MULTIPLIER is that extra.
 *    Came up short -> raise it. Went too far -> lower it. The suggested value is printed.
 *    LATERAL_MULTIPLIER is read when the drive is created, so re-run (stop/start) after changing it.
 *
 *  FEEDBACK = true (normal) -> final check.
 */
@Config
@Autonomous(name = "RR 5b StrafeTest", group = "RR Tuning")
public class StrafeTest extends RRTuningOpMode
{
    public static double DISTANCE = 60; // in
    public static boolean FEEDBACK = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d()).strafeRight(DISTANCE).build();

        tel.addLine(FEEDBACK ? "FEEDBACK ON: final check" : "FEEDBACK OFF: tuning LATERAL_MULTIPLIER");
        tel.update();
        waitForStart();
        if (isStopRequested()) return;

        try
        {
            setFeedbackEnabled(FEEDBACK);
            drive.followTrajectoryAsync(trajectory);
            updateUntilIdle();
        }
        finally
        {
            setFeedbackEnabled(true);
        }

        double actual = -drive.getPoseEstimate().getY(); // strafing right is -y
        if (!FEEDBACK && Math.abs(actual) > 1)
        {
            tel.addLine(String.format(Locale.US, "Suggested LATERAL_MULTIPLIER = %.3f (now %.3f)",
                    RobotConstants.LATERAL_MULTIPLIER * DISTANCE / actual,
                    RobotConstants.LATERAL_MULTIPLIER));
        }
        StraightTest.reportEndPose(this, 0, -DISTANCE);
    }
}
