package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Dumper;
import org.firstinspires.ftc.teamcode.robot.Lifter;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.EL_MAX_ENCODER;

@TeleOp(name = "Testlifter", group = "Test")
@Disabled
public class Testlifter extends InitLinearOpMode {
    private static final int CYCLE_MS = 20;       // period of each cycle

    // Define class members
    private final ElapsedTime period  = new ElapsedTime();

    private static final String TAG = "SJH_TLF";
    protected static final boolean VERBOSE = RobotConstants.logVerbose;

    @Override
    public void runOpMode() {
        initCommon(this);

        Lifter lifter = new Lifter("slide1","slide2" ,hardwareMap);

        lifter.init();
        Dumper dumper = new Dumper(hardwareMap);
        dumper.init();
        int p;
        final double srvIncr = 0.05;

        // Wait for the start button
        dashboard.displayText(0, "Press Start to run Motors.");

        while (!isStarted()) {
            lifter.update();

            String lStr = lifter.toString();
            dashboard.displayText(3, lStr);
            if(VERBOSE){RobotLog.dd(TAG, lStr);}

            waitForTick(CYCLE_MS);
        }
        waitForStart();

        // Ramp motor speeds till stop pressed.
        boolean usePower = false;
        while (opModeIsActive()) {
            p = 0;

            if(lifter != null) lifter.update();
            if(dumper != null) dumper.update();
            gpad1.update();
            double lftPwr = -gpad1.value(ManagedGamepad.AnalogInput.L_STICK_Y);
            boolean dump = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
            boolean stow = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean grab = gpad1.just_pressed(ManagedGamepad.Button.A);
            boolean low  = gpad1.just_pressed(ManagedGamepad.Button.B);
            boolean mid  = gpad1.just_pressed(ManagedGamepad.Button.X);
            boolean high = gpad1.just_pressed(ManagedGamepad.Button.Y);
            boolean lsrv = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean rsrv = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean stop = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean tglS = gpad1.just_pressed(ManagedGamepad.Button.R_TRIGGER);

            String lStr = "";
            if(dumper != null)
            {
                if      (dump) dumper.setDumpPos(RobotConstants.DP_DUMP_DEPLOY);
                else if (stow) dumper.setDumpPos(RobotConstants.DP_DUMP_STOW);
                else if (lsrv) dumper.adjDumpPos(-srvIncr);
                else if (rsrv) dumper.adjDumpPos(srvIncr);
                lStr += dumper.toString();
            }


            if(lifter != null)
            {
                if (tglS)
                {
                    usePower = !usePower;
                    if (usePower) lifter.setMode(RUN_WITHOUT_ENCODER);
                    else lifter.setMode(RUN_USING_ENCODER);
                }

                if (usePower)
                {
                    lifter.setLiftPwr(lftPwr);
                }
                else
                {
                    if      (stow) lifter.setLiftPos(RobotConstants.LF_ARM_ROT_STOW);
                    else if (low)  lifter.setLiftPos(RobotConstants.LF_ARM_ROT_LOW);
                    else if (mid)  lifter.setLiftPos(RobotConstants.LF_ARM_ROT_MID);
                    else if (high) lifter.setLiftPos(RobotConstants.LF_ARM_ROT_HIGH);
                    else if (grab) lifter.setLiftPos(RobotConstants.LF_ARM_ROT_GRAB);
                    else lifter.setLiftSpd(lftPwr, EL_MAX_ENCODER);
                }

                if (stop) lifter.setLiftPwr(0.0);

                // Display the current value
                lStr += lifter.toString();
            }
            dashboard.displayText(p++, lStr);
            dashboard.displayText(p++, "usePower: " + usePower);
            if(VERBOSE){RobotLog.dd(TAG, lStr);}

            dashboard.displayText(p++, "Press Stop to end test.");
            dashboard.displayText(p++, "Decr grip : Dpad left");
            dashboard.displayText(p++, "Incr grip : Dpad right");
            dashboard.displayText(p++, "Open  grp : L_BUMP");
            dashboard.displayText(p++, "Close grp : R_BUMP");
            dashboard.displayText(p++, "Toggle spdMod : R_TRG");
            dashboard.displayText(p++, "Arm Speed : L_Stick_Y");
            dashboard.displayText(p++, "Stow : A");
            dashboard.displayText(p++, "Grab : B");
            dashboard.displayText(p++, "Hold : X");
            dashboard.displayText(p++, "Drop : Y");

            dashboard.displayText(p, "Zero power : Dpad down");

            waitForTick(CYCLE_MS);
        }

        lifter.setLiftPwr(0.0);
        dashboard.displayText(1, "Done.");
    }

    public void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

}