package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.MotorComponent;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

import java.util.Locale;

@Config
@TeleOp(name = "Testintake", group = "Test")
//@Disabled
public class TestIntake extends InitLinearOpMode
{
    private static final double INCREMENT = 0.10;     // amount to step motor each CYCLE_MS cycle
    private static final int     CYCLE_MS = 20;       // period of each cycle
    private static final double   MAX_FWD =  1.0;     // Maximum FWD power applied to motor
    private static final double   MAX_REV = -1.0;     // Maximum REV power applied to motor

    // Define class members
    private double power = 0;
    private MotorComponent intake;
    
    private int prevnumTrig = 0;

    private FtcDashboard dbd;

    private static final String TAG = "SJH_TIN";
    protected static final boolean VERBOSE = RobotConstants.logVerbose;

    private void doLogging()
    {
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("pos", intake.getCurEnc());
        packet.put("spd", intake.getCurSpd());
        dbd.sendTelemetryPacket(packet);

        String intStr = intake.toString();
       if(VERBOSE){ RobotLog.dd(TAG, intStr);}

        int p = 0;

        // Wait for the start button
        dashboard.displayText(p++, "Press Start to run Motors.");
        dashboard.displayText(p++, "Press Stop to end test.");
        dashboard.displayText(p++, "Incr power : Dpad up");
        dashboard.displayText(p++, "Decr power : Dpad down");
        dashboard.displayText(p++, "Zero power : Dpad right");
        dashboard.displayText(p++, intStr);
        // Display the current value
        String mpStr = String.format(Locale.US,"Cmd Power %4.2f", power);
        dashboard.displayText(p++, mpStr);
        if(VERBOSE){RobotLog.dd(TAG, mpStr);}

        dashboard.displayText(p,
                              String.format(Locale.US,"ActionTriggered %s %d",
                                            intake.getActionTriggered(),
                                            intake.getNumActionEvents()));
    }

    private void update()
    {
        gpad1.update();
        intake.update();
    }

    @Override
    public void runOpMode()
    {
        initCommon(this);
        dbd = CommonUtil.getInstance().getFtcDashboard();

        RobotConstants.init(RobotConstants.Chassis.B7253);

        intake = new MotorComponent("intake", hardwareMap);
        intake.init(RobotConstants.IN_MOTOR_MOT, RobotConstants.IN_GEAR);
        intake.setDir(RobotConstants.IN_DIR);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive())
        {
            update();
            doLogging();

            boolean step_up    = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean step_down  = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean zeroize    = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);

            if(step_up && power < MAX_FWD)         power += INCREMENT;
            else if(step_down && power > MAX_REV)  power -= INCREMENT;
            else if(zeroize)                       power = 0.0;

            if (intake.getNumActionEvents() != prevnumTrig)
            {
                power = 0;
            }
            prevnumTrig = intake.getNumActionEvents();

            intake.moveAtRate(power);

            sleep(CYCLE_MS);
        }

        intake.stop();

        dashboard.displayText(  1, "Done." );
    }
}
