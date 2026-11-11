package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Sanity check: turn ANGLE degrees in place and report the end pose. */
@Config
@Autonomous(name = "RR 5c TurnTest", group = "RR Tuning")
public class TurnTest extends RRTuningOpMode
{
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();

        waitForStart();
        if (isStopRequested()) return;

        drive.turnAsync(Math.toRadians(ANGLE));
        updateUntilIdle();

        tel.addData("target heading (deg)", ANGLE);
        StraightTest.reportEndPose(this, 0, 0);
    }
}
