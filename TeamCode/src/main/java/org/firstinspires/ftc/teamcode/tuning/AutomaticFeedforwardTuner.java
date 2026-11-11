package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.util.LoggingUtil;
import org.firstinspires.ftc.teamcode.util.RegressionUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Step 3a. Ramp-regression feedforward tuner (the Road Runner 0.x equivalent of RR 1.0's
 * ForwardRampLogger). Slowly ramps forward power, fits kV (and optionally kStatic) from measured
 * Pinpoint position, then optionally does a constant-power run to fit kA.
 *
 * Use the results as a starting point in RobotConstants (kV, kStatic, kA), then refine with
 * ManualFeedforwardTuner. Raw data is saved to /sdcard/FIRST/RoadRunner/ on the hub.
 *
 * Tune with a charged battery: MecanumDriveLRR does not voltage-compensate its feedforward.
 */
@Config
@Autonomous(name = "RR 3a AutomaticFeedforwardTuner", group = "RR Tuning")
public class AutomaticFeedforwardTuner extends RRTuningOpMode
{
    public static double MAX_POWER = 0.7;
    public static double DISTANCE = 100; // in

    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        NanoClock clock = NanoClock.system();

        if (RobotConstants.RUN_USING_ENCODER)
        {
            tel.addLine("WARNING: RUN_USING_ENCODER is true; feedforward is normally tuned with it false.");
        }
        tel.addLine("Press play to begin the feedforward tuning routine");
        tel.update();
        waitForStart();
        if (isStopRequested()) return;

        boolean fitIntercept = askYesNo("Would you like to fit kStatic?");
        if (isStopRequested()) return;

        tel.addLine(String.format(Locale.US,
                "Place the robot with at least %.0f in of room in front of it", DISTANCE));
        tel.addLine("Press (Y/Δ) to begin");
        tel.update();
        if (!waitForButton(() -> gamepad1.y)) return;

        tel.addLine("Running...");
        tel.update();

        double maxVel = RobotConstants.rpmToVelocity(RobotConstants.DT_MAX_RPM);
        double finalVel = MAX_POWER * maxVel;
        double accel = (finalVel * finalVel) / (2.0 * DISTANCE);
        double rampTime = Math.sqrt(2.0 * DISTANCE / accel);

        List<Double> timeSamples = new ArrayList<>();
        List<Double> positionSamples = new ArrayList<>();
        List<Double> powerSamples = new ArrayList<>();

        drive.setPoseEstimate(new Pose2d());

        double startTime = clock.seconds();
        while (!isStopRequested())
        {
            double elapsedTime = clock.seconds() - startTime;
            if (elapsedTime > rampTime) break;
            double vel = accel * elapsedTime;
            double power = vel / maxVel;

            timeSamples.add(elapsedTime);
            positionSamples.add(drive.getPoseEstimate().getX());
            powerSamples.add(power);

            drive.setDrivePower(new Pose2d(power, 0.0, 0.0));
            drive.updatePoseEstimate();
        }
        drive.setDrivePower(new Pose2d());
        if (isStopRequested()) return;

        RegressionUtil.RampResult rampResult = RegressionUtil.fitRampData(
                timeSamples, positionSamples, powerSamples, fitIntercept,
                LoggingUtil.getLogFile(String.format(Locale.US,
                        "DriveRampRegression-%d.csv", System.currentTimeMillis())));

        String rampLine = fitIntercept
                ? String.format(Locale.US, "kV = %.5f, kStatic = %.5f (R^2 = %.2f)",
                        rampResult.kV, rampResult.kStatic, rampResult.rSquare)
                : String.format(Locale.US, "kV = %.5f (R^2 = %.2f)",
                        rampResult.kV, rampResult.rSquare);

        tel.addLine("Quasi-static ramp up test complete");
        tel.addLine(rampLine);
        boolean fitAccelFF = askYesNo("Would you like to fit kA?");

        if (fitAccelFF && !isStopRequested())
        {
            tel.addLine(rampLine);
            tel.addLine("Place the robot back in its starting position");
            tel.addLine("Press (Y/Δ) to continue");
            tel.update();
            if (!waitForButton(() -> gamepad1.y)) return;

            tel.addLine("Running...");
            tel.update();

            double maxPowerTime = DISTANCE / maxVel;

            timeSamples.clear();
            positionSamples.clear();
            powerSamples.clear();

            drive.setPoseEstimate(new Pose2d());
            drive.setDrivePower(new Pose2d(MAX_POWER, 0.0, 0.0));

            startTime = clock.seconds();
            while (!isStopRequested())
            {
                double elapsedTime = clock.seconds() - startTime;
                if (elapsedTime > maxPowerTime) break;

                timeSamples.add(elapsedTime);
                positionSamples.add(drive.getPoseEstimate().getX());
                powerSamples.add(MAX_POWER);

                drive.updatePoseEstimate();
            }
            drive.setDrivePower(new Pose2d());
            if (isStopRequested()) return;

            RegressionUtil.AccelResult accelResult = RegressionUtil.fitAccelData(
                    timeSamples, positionSamples, powerSamples, rampResult,
                    LoggingUtil.getLogFile(String.format(Locale.US,
                            "DriveAccelRegression-%d.csv", System.currentTimeMillis())));

            tel.addLine("Constant power test complete");
            tel.addLine(rampLine);
            tel.addLine(String.format(Locale.US, "kA = %.5f (R^2 = %.2f)",
                    accelResult.kA, accelResult.rSquare));
        }
        else
        {
            tel.addLine(rampLine);
        }
        tel.addLine(String.format(Locale.US, "Current: kV=%.5f kA=%.5f kStatic=%.5f",
                RobotConstants.kV, RobotConstants.kA, RobotConstants.kStatic));
        tel.update();

        while (!isStopRequested()) idle();
    }
}
