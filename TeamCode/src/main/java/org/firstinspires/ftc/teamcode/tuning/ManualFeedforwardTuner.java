package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

/**
 * Step 3b. Manual feedforward tuner (RR 0.x ManualFeedforwardTuner == RR 1.0 ManualFeedforwardTuner).
 *
 * The robot drives back and forth DISTANCE inches on a motion profile using ONLY feedforward.
 * Open FTC Dashboard, graph "targetVelocity" and "measuredVelocity", and edit RobotConstants
 * kV / kA / kStatic live until the two curves overlap:
 *   kV     - matches the flat (cruise) part of the curve
 *   kStatic - helps it start moving / stop lagging at low speed
 *   kA     - matches the ramps (accel/decel)
 *
 * Set AXIS = STRAFE to tune LATERAL_MULTIPLIER the same way (the RR 1.0 "lateral" step): the robot
 * strafes and you adjust RobotConstants.LATERAL_MULTIPLIER until measured strafe velocity matches.
 *
 * Press Y to take manual control (left stick translate, right stick turn) and reposition;
 * press B to resume tuning. Tune with a charged battery (no voltage compensation in MecanumDriveLRR).
 */
@Config
@Autonomous(name = "RR 3b ManualFeedforwardTuner", group = "RR Tuning")
public class ManualFeedforwardTuner extends RRTuningOpMode
{
    public static double DISTANCE = 72; // in
    public static Axis AXIS = Axis.FORWARD;

    public enum Axis { FORWARD, STRAFE }

    private enum Mode { DRIVER_MODE, TUNING_MODE }

    private static MotionProfile generateProfile(boolean movingForward)
    {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
                RobotConstants.MAX_VEL, RobotConstants.MAX_ACCEL);
    }

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
        tel.addLine("Ready! Graph targetVelocity vs measuredVelocity in FTC Dashboard.");
        tel.update();
        waitForStart();
        if (isStopRequested()) return;

        Mode mode = Mode.TUNING_MODE;
        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();

        while (!isStopRequested())
        {
            tel.addData("mode", mode);
            tel.addData("axis", AXIS);

            switch (mode)
            {
                case TUNING_MODE:
                {
                    if (gamepad1.y) mode = Mode.DRIVER_MODE;

                    double profileTime = clock.seconds() - profileStart;
                    if (profileTime > activeProfile.duration())
                    {
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                        profileTime = 0;
                    }

                    MotionState motionState = activeProfile.get(profileTime);
                    double targetPower = Kinematics.calculateMotorFeedforward(
                            motionState.getV(), motionState.getA(),
                            RobotConstants.kV, RobotConstants.kA, RobotConstants.kStatic);

                    if (AXIS == Axis.FORWARD)
                    {
                        drive.setDrivePower(new Pose2d(targetPower, 0, 0));
                    }
                    else
                    {
                        // Strafe left using the LIVE dashboard LATERAL_MULTIPLIER (the drive's own
                        // kinematics captured the value when it was constructed).
                        // Wheel order: leftFront, leftRear, rightRear, rightFront.
                        double p = targetPower * RobotConstants.LATERAL_MULTIPLIER;
                        drive.setMotorPowers(-p, p, -p, p);
                    }
                    drive.updatePoseEstimate();

                    Pose2d poseVelo = drive.getPoseVelocity();
                    double currentVelo = 0.0;
                    if (poseVelo != null)
                    {
                        currentVelo = (AXIS == Axis.FORWARD) ? poseVelo.getX() : poseVelo.getY();
                    }

                    tel.addData("targetVelocity", motionState.getV());
                    tel.addData("measuredVelocity", currentVelo);
                    tel.addData("error", motionState.getV() - currentVelo);
                    break;
                }
                case DRIVER_MODE:
                    if (gamepad1.b)
                    {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(true);
                        profileStart = clock.seconds();
                    }
                    drive.setWeightedDrivePower(new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x));
                    drive.updatePoseEstimate();
                    break;
            }
            tel.update();
        }
        drive.setDrivePower(new Pose2d());
    }
}
