package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Step 1. Drive the robot around with gamepad1 and check that the Pinpoint pose tracks reality:
 *  - push forward  -> x increases
 *  - push left     -> y increases
 *  - turn CCW      -> heading increases
 * Also check that "robot vel" x is positive when driving forward no matter which way the robot faces.
 * If a sign is wrong, fix the Pinpoint encoder directions / offsets in PinpointLocalizer.
 *
 * Controls: left stick = translate, right stick X = turn.
 */
@TeleOp(name = "RR 1 LocalizationTest", group = "RR Tuning")
public class LocalizationTest extends RRTuningOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initTuningDrive();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tel.addLine("Left stick = translate, right stick X = turn");
        tel.update();
        waitForStart();

        while (opModeIsActive())
        {
            drive.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x));
            drive.update();

            Pose2d pose = drive.getPoseEstimate();
            Pose2d vel = drive.getPoseVelocity();
            tel.addData("x", pose.getX());
            tel.addData("y", pose.getY());
            tel.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
            if (vel != null)
            {
                tel.addData("robot vel x (in/s)", vel.getX());
                tel.addData("robot vel y (in/s)", vel.getY());
                tel.addData("heading vel (deg/s)", Math.toDegrees(vel.getHeading()));
            }
            tel.update();
        }
    }
}
