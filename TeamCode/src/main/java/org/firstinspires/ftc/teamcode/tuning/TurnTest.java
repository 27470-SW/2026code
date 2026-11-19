package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

import java.util.Locale;

/**
 * Turn ANGLE degrees in place. Two uses (see tuning/README.md):
 *
 *  FEEDBACK = false (feedforward only) -> quick DT_TRACK_WIDTH check (TrackWidthTuner averages
 *    several turns and is more accurate). Under-turned -> raise DT_TRACK_WIDTH, over-turned -> lower.
 *
 *  FEEDBACK = true (normal) -> final check.
 */
@Config
@Autonomous(name = "RR 5c TurnTest", group = "RR Tuning")
public class TurnTest extends RRTuningOpMode
{
    public static double ANGLE = 90; // deg
    public static boolean FEEDBACK = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();

        tel.addLine(FEEDBACK ? "FEEDBACK ON: final check" : "FEEDBACK OFF: checking DT_TRACK_WIDTH");
        tel.update();
        waitForStart();
        if (isStopRequested()) return;

        try
        {
            setFeedbackEnabled(FEEDBACK);
            drive.turnAsync(Math.toRadians(ANGLE));
            updateUntilIdle();
        }
        finally
        {
            setFeedbackEnabled(true);
        }

        double actual = Math.toDegrees(drive.getPoseEstimate().getHeading());
        if (!FEEDBACK && Math.abs(actual) > 1)
        {
            tel.addLine(String.format(Locale.US, "Suggested DT_TRACK_WIDTH = %.2f (now %.2f)",
                    RobotConstants.DT_TRACK_WIDTH * ANGLE / actual, RobotConstants.DT_TRACK_WIDTH));
        }
        tel.addData("target heading (deg)", ANGLE);
        StraightTest.reportEndPose(this, 0, 0);
    }
}
