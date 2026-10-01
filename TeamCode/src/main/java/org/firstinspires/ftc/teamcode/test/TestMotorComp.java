package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.MotorComponent;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

@TeleOp(name = "TestMotorComp", group = "Test")
//@Disabled
public class TestMotorComp extends InitLinearOpMode
{
    private static final int    CYCLE_MS = 20;       // period of each cycle
    private static final double[] LEVS = {0.0, 6.1, 12.75};
    private static double max_ips = 18.0;
    private static final double ips_step = 0.1;

    private static final String TAG = "SJH_TSC";
    protected static final boolean VERBOSE = RobotConstants.logVerbose;

    @Override
    public void runOpMode()
    {
        initCommon(this);

        MotorComponent mc = new MotorComponent("elev", hardwareMap);
        mc.init(18);
        mc.setLevelOffset(LEVS);
        mc.setDir(DcMotorSimple.Direction.REVERSE);
        mc.setMaxSpeed(1);

        // Wait for the start button
        dashboard.displayText(0, "Press Start to run MotorTest.");

        waitForStart();

        int p;
        int l = 0;

        while(opModeIsActive())
        {
            p=0;

            gpad1.update();
            mc.update();

            boolean lev_up = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean lev_dn = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean m1     = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean m2     = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            double  spd    = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
            boolean runTst = gpad1.pressed(ManagedGamepad.Button.R_TRIGGER);
            boolean rstTst = gpad1.just_released(ManagedGamepad.Button.R_TRIGGER);
            boolean incIps = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean decIps = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);


            if(lev_up && l < LEVS.length)  {l++; mc.moveToLevel(l, 0.5);}
            else if(lev_dn && l >= 0)      {l--; mc.moveToLevel(l, 0.1);}
            else if(m1)                    {mc.moveTo(2.0, 1.0);}
            else if(m2)                    {mc.moveTo(5.0, 0.25);}
            else                           {mc.moveAtRate(spd);}

            if(runTst) mc.findCfg();
            if(rstTst) mc.resetTest();

            if(incIps) {max_ips += ips_step; mc.setMaxIps(max_ips);}
            if(decIps) {max_ips -= ips_step; mc.setMaxIps(max_ips);}

            // Display the current value
            String mpStr = mc.toString();
            dashboard.displayText(p++, mpStr);
            if(VERBOSE){RobotLog.dd(TAG, mpStr);}

            dashboard.displayText(p++,"Press Stop to end test.");
            dashboard.displayText(p++,"Level     up : Dpad up");
            dashboard.displayText(p++,"LevelRate dn : Dpad down");
            dashboard.displayText(p,  "MoveTo       : Dpad left");
            dashboard.displayText(p,  "MoveToRate   : Dpad right");
            dashboard.displayText(p,  "MoveAtRate   : R_STICK_Y");

            sleep(CYCLE_MS);
        }

        dashboard.displayText(  1, "Done." );
    }
}
