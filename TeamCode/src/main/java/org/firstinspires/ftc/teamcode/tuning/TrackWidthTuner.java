package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

import java.util.Locale;

/**
 * Step 4. Effective track width (RR 1.0 equivalent: AngularRampLogger).
 *
 * Turns ANGLE degrees NUM_TRIALS times using feedforward only and compares the commanded turn to
 * the turn the Pinpoint measured. Put the "Effective track width" result in
 * RobotConstants.DT_TRACK_WIDTH. Run after kV/kA/kStatic are tuned. If the robot over-turns the
 * result is smaller than the current value, if it under-turns it is larger.
 *
 * HEADING_PID is zeroed while this runs (so feedback can't hide the error) and restored afterwards.
 */
@Config
@Autonomous(name = "RR 4 TrackWidthTuner", group = "RR Tuning")
public class TrackWidthTuner extends RRTuningOpMode
{
    public static double ANGLE = 180; // deg
    public static int NUM_TRIALS = 5;
    public static int DELAY = 1000; // ms

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();

        tel.addLine("Press play to begin the track width tuner routine");
        tel.addLine("Make sure the robot has room to turn");
        tel.update();
        waitForStart();
        if (isStopRequested()) return;

        tel.addLine("Running...");
        tel.update();

        // The follower holds a reference to this object, so zeroing the fields disables heading feedback.
        double kP = RobotConstants.HEADING_PID.kP;
        double kI = RobotConstants.HEADING_PID.kI;
        double kD = RobotConstants.HEADING_PID.kD;
        RobotConstants.HEADING_PID.kP = 0;
        RobotConstants.HEADING_PID.kI = 0;
        RobotConstants.HEADING_PID.kD = 0;

        MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
        try
        {
            for (int i = 0; i < NUM_TRIALS && !isStopRequested(); i++)
            {
                drive.setPoseEstimate(new Pose2d());

                // it is important to handle heading wraparounds
                double headingAccumulator = 0;
                double lastHeading = 0;

                drive.turnAsync(Math.toRadians(ANGLE));

                while (!isStopRequested() && drive.isBusy())
                {
                    double heading = drive.getPoseEstimate().getHeading();
                    headingAccumulator += Angle.normDelta(heading - lastHeading);
                    lastHeading = heading;
                    drive.update();
                }
                drive.setDrivePower(new Pose2d());
                if (isStopRequested()) break;

                double trackWidth = RobotConstants.DT_TRACK_WIDTH * Math.toRadians(ANGLE) / headingAccumulator;
                trackWidthStats.add(trackWidth);
                tel.addLine(String.format(Locale.US, "Trial %d: measured %.1f deg -> track width %.2f",
                        i + 1, Math.toDegrees(headingAccumulator), trackWidth));
                tel.update();

                sleep(DELAY);
            }
        }
        finally
        {
            RobotConstants.HEADING_PID.kP = kP;
            RobotConstants.HEADING_PID.kI = kI;
            RobotConstants.HEADING_PID.kD = kD;
            drive.cancelFollowing();
            drive.setDrivePower(new Pose2d());
        }
        if (isStopRequested()) return;

        tel.addLine("Tuning complete");
        tel.addLine(String.format(Locale.US, "Effective track width = %.2f (SE = %.3f)",
                trackWidthStats.getMean(),
                trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        tel.addLine(String.format(Locale.US, "Current DT_TRACK_WIDTH = %.2f", RobotConstants.DT_TRACK_WIDTH));
        tel.update();

        while (!isStopRequested()) idle();
    }
}
