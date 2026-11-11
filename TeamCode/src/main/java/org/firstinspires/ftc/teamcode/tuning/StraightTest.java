package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

import java.util.Locale;

/**
 * Drive forward DISTANCE inches. Two uses (see tuning/README.md):
 *
 *  FEEDBACK = false (feedforward only) -> tunes kV. With the PIDs off nothing corrects the robot,
 *    so where it stops shows how good kV is. Power = kV * velocity, so: came up short -> raise kV,
 *    went too far -> lower kV. The suggested value is printed at the end.
 *
 *  FEEDBACK = true (normal) -> final check. Should end within ~0.5 in of the target.
 *
 * Always measure the real distance with a tape measure and compare it to the reported x: if they
 * disagree, the Pinpoint setup is wrong (pod type, encoder direction), not the tuning.
 */
@Config
@Autonomous(name = "RR 5a StraightTest", group = "RR Tuning")
public class StraightTest extends RRTuningOpMode
{
    public static double DISTANCE = 100; // in
    public static boolean FEEDBACK = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d()).forward(DISTANCE).build();

        tel.addLine(FEEDBACK ? "FEEDBACK ON: final check" : "FEEDBACK OFF: tuning kV");
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

        double actual = drive.getPoseEstimate().getX();
        if (!FEEDBACK && Math.abs(actual) > 1)
        {
            tel.addLine(String.format(Locale.US, "Suggested kV = %.5f (now %.5f)",
                    RobotConstants.kV * DISTANCE / actual, RobotConstants.kV));
        }
        reportEndPose(this, DISTANCE, 0);
    }

    static void reportEndPose(RRTuningOpMode op, double targetX, double targetY)
    {
        Pose2d pose = op.drive.getPoseEstimate();
        op.tel.addData("target x / y", "%.1f / %.1f", targetX, targetY);
        op.tel.addData("finalX", pose.getX());
        op.tel.addData("finalY", pose.getY());
        op.tel.addData("finalHeading (deg)", Math.toDegrees(pose.getHeading()));
        op.tel.addLine("Compare with a tape measure: they should agree.");
        op.tel.update();
        while (!op.isStopRequested() && op.opModeIsActive()) op.idle();
    }
}
