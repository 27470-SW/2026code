package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;

/**
 * Base class for the Road Runner 0.x tuning OpModes.
 *
 * Extending InitLinearOpMode means RobotConstants.init(chassis) has already run (static block), so
 * the tuners start from exactly the same drive constants the competition OpModes use. After that,
 * every RobotConstants value is live-editable from FTC Dashboard (http://192.168.43.1:8080/dash)
 * and stays edited until the Robot Controller app restarts, or until an OpMode that calls
 * RobotConstants.init() again (e.g. Decode Auton) is initialized.
 *
 * Only the drivetrain is created (no MecanumBot), so the tuners don't move any mechanisms.
 */
public abstract class RRTuningOpMode extends InitLinearOpMode
{
    private static final String TAG = "RR_TUNE";

    protected MecanumDriveLRR drive;
    /** Telemetry that goes to both the Driver Station and FTC Dashboard. */
    protected Telemetry tel;

    protected MecanumDriveLRR initTuningDrive()
    {
        initCommon(this);

        // HalDashboard (created by initCommon) turns autoClear off and adds 16 fixed lines;
        // the tuners use plain telemetry instead.
        telemetry.clearAll();
        telemetry.setAutoClear(true);
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Same IMU name/orientation as BasicBot. Heading comes from the Pinpoint, but
        // MecanumDriveLRR() with no IMU would look for an "imu-new" device instead.
        IMU imu = null;
        try
        {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        }
        catch (Exception e)
        {
            RobotLog.ww(TAG, "No IMU named 'imu'; MecanumDriveLRR will look for 'imu-new'");
        }

        drive = new MecanumDriveLRR(imu);
        drive.setPoseEstimate(new Pose2d());
        return drive;
    }

    private double[] savedGains = null;

    /**
     * Turn the follower's feedback (TRANSLATIONAL_PID and HEADING_PID) off or back on.
     * The follower holds references to these objects, so zeroing their fields disables feedback
     * immediately. Always re-enable in a finally block: the values are static and would otherwise
     * stay zero for every OpMode until the app restarts.
     */
    protected void setFeedbackEnabled(boolean enabled)
    {
        if (!enabled && savedGains == null)
        {
            savedGains = new double[] {
                    RobotConstants.TRANSLATIONAL_PID.kP, RobotConstants.TRANSLATIONAL_PID.kI,
                    RobotConstants.TRANSLATIONAL_PID.kD,
                    RobotConstants.HEADING_PID.kP, RobotConstants.HEADING_PID.kI,
                    RobotConstants.HEADING_PID.kD };
            RobotConstants.TRANSLATIONAL_PID.kP = 0;
            RobotConstants.TRANSLATIONAL_PID.kI = 0;
            RobotConstants.TRANSLATIONAL_PID.kD = 0;
            RobotConstants.HEADING_PID.kP = 0;
            RobotConstants.HEADING_PID.kI = 0;
            RobotConstants.HEADING_PID.kD = 0;
        }
        else if (enabled && savedGains != null)
        {
            RobotConstants.TRANSLATIONAL_PID.kP = savedGains[0];
            RobotConstants.TRANSLATIONAL_PID.kI = savedGains[1];
            RobotConstants.TRANSLATIONAL_PID.kD = savedGains[2];
            RobotConstants.HEADING_PID.kP = savedGains[3];
            RobotConstants.HEADING_PID.kI = savedGains[4];
            RobotConstants.HEADING_PID.kD = savedGains[5];
            savedGains = null;
        }
    }

    /** Run drive.update() until the current trajectory/sequence ends or the OpMode stops. */
    protected void updateUntilIdle()
    {
        while (opModeIsActive() && !isStopRequested() && drive.isBusy())
        {
            drive.update();
        }
        if (isStopRequested())
        {
            drive.cancelFollowing();
            drive.setDrivePower(new Pose2d());
        }
    }

    /** Block until the given gamepad1 button is pressed and released. */
    protected boolean waitForButton(ButtonCheck button)
    {
        while (!isStopRequested() && !button.pressed()) idle();
        while (!isStopRequested() && button.pressed()) idle();
        return !isStopRequested();
    }

    /** Ask a yes/no question on gamepad1: Y = yes, B = no. */
    protected boolean askYesNo(String question)
    {
        tel.addLine(question);
        tel.addLine("Press (Y/Δ) for yes, (B/O) for no");
        tel.update();
        while (!isStopRequested())
        {
            if (gamepad1.y) { waitForButton(() -> gamepad1.y); return true; }
            if (gamepad1.b) { waitForButton(() -> gamepad1.b); return false; }
            idle();
        }
        return false;
    }

    protected interface ButtonCheck
    {
        boolean pressed();
    }
}
