package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Sanity check: follow a straight line forward DISTANCE inches and report the end pose. */
@Config
@Autonomous(name = "RR 5a StraightTest", group = "RR Tuning")
public class StraightTest extends RRTuningOpMode
{
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d()).forward(DISTANCE).build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectoryAsync(trajectory);
        updateUntilIdle();

        reportEndPose(this, DISTANCE, 0);
    }

    static void reportEndPose(RRTuningOpMode op, double targetX, double targetY)
    {
        Pose2d pose = op.drive.getPoseEstimate();
        op.tel.addData("target x / y", "%.1f / %.1f", targetX, targetY);
        op.tel.addData("finalX", pose.getX());
        op.tel.addData("finalY", pose.getY());
        op.tel.addData("finalHeading (deg)", Math.toDegrees(pose.getHeading()));
        op.tel.addLine("Measure the real distance with a tape measure too.");
        op.tel.update();
        while (!op.isStopRequested() && op.opModeIsActive()) op.idle();
    }
}
