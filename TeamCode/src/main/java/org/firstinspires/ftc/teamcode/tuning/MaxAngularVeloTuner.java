package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

/**
 * Step 2b. Spins in place at full power for RUNTIME seconds and reports the top angular speed.
 * Set RobotConstants.MAX_ANG_VEL to at most ~80% of the result.
 */
@Config
@Autonomous(name = "RR 2b MaxAngularVeloTuner", group = "RR Tuning")
public class MaxAngularVeloTuner extends RRTuningOpMode
{
    public static double RUNTIME = 4.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tel.addLine("The robot will SPIN at full power for " + RUNTIME + " s.");
        tel.addLine("Make sure there is room around it. Press start when ready.");
        tel.update();
        waitForStart();
        if (isStopRequested()) return;

        double maxAngVelocity = 0.0;
        drive.setDrivePower(new Pose2d(0, 0, 1));
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < RUNTIME)
        {
            drive.updatePoseEstimate();
            Pose2d vel = drive.getPoseVelocity();
            if (vel != null) maxAngVelocity = Math.max(maxAngVelocity, Math.abs(vel.getHeading()));
        }
        drive.setDrivePower(new Pose2d());

        tel.addData("Max angular velocity (rad/s)", "%.3f", maxAngVelocity);
        tel.addData("Max angular velocity (deg/s)", "%.1f", Math.toDegrees(maxAngVelocity));
        tel.addData("Suggested MAX_ANG_VEL (80%, deg/s)", "%.1f", Math.toDegrees(0.8 * maxAngVelocity));
        tel.addData("Current MAX_ANG_VEL (deg/s)", "%.1f", Math.toDegrees(RobotConstants.MAX_ANG_VEL));
        tel.update();
        while (opModeIsActive()) idle();
    }
}
