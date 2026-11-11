package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

/**
 * Step 2a. Drives forward at full power for RUNTIME seconds and reports the top speed seen.
 * Set RobotConstants.MAX_VEL to at most ~80% of the result. Needs a long clear runway.
 */
@Config
@Autonomous(name = "RR 2a MaxVelocityTuner", group = "RR Tuning")
public class MaxVelocityTuner extends RRTuningOpMode
{
    public static double RUNTIME = 2.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tel.addLine("The robot will drive FORWARD at full power for " + RUNTIME + " s.");
        tel.addLine("Make sure there is enough room. Press start when ready.");
        tel.update();
        waitForStart();
        if (isStopRequested()) return;

        double maxVelocity = 0.0;
        drive.setDrivePower(new Pose2d(1, 0, 0));
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < RUNTIME)
        {
            drive.updatePoseEstimate();
            Pose2d vel = drive.getPoseVelocity();
            if (vel != null) maxVelocity = Math.max(maxVelocity, vel.vec().norm());
        }
        drive.setDrivePower(new Pose2d());

        tel.addData("Max velocity (in/s)", "%.2f", maxVelocity);
        tel.addData("Suggested MAX_VEL (80%)", "%.1f", 0.8 * maxVelocity);
        tel.addData("Current MAX_VEL", "%.1f", RobotConstants.MAX_VEL);
        tel.addData("Theoretical free speed (in/s)", "%.1f", RobotConstants.DT_MAX_IPS);
        tel.update();
        while (opModeIsActive()) idle();
    }
}
