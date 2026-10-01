package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.ServoComponent;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

@TeleOp(name = "TestServoComp", group = "Test")
//@Disabled
public class TestServoComp extends InitLinearOpMode
{
    private static final int    CYCLE_MS = 20;       // period of each cycle
    private static final double MAX_SRV_POS = 1.0;
    private static final double MIN_SRV_POS = 0.60;
    private static final double[] LEVS = {0.0, 0.25, 0.5, 0.75, 1.0};

    private static final String TAG = "SJH_TSC";

    @Override
    public void runOpMode()
    {
        initCommon(this);

        ServoComponent sc = new ServoComponent("dumper", hardwareMap);
        sc.init();

        sc.scaleRange(MIN_SRV_POS, MAX_SRV_POS);
        sc.setLevelOffset(LEVS);

        // Wait for the start button
        dashboard.displayText(0, "Press Start to run ServoTest.");

        waitForStart();

        int p;
        int l = 0;
        // Ramp motor speeds till stop pressed.
        while(opModeIsActive())
        {
            p=0;

            gpad1.update();
            sc.update();

            boolean lev_up =  gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean lev_dn =  gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean m1     =  gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean m2     =  gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            double  spd    = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);

            if(lev_up && (l+1) < LEVS.length)  {l++; sc.moveToLevel(l);}
            else if(lev_dn && l > 0)           {l--; sc.moveToLevel(l, 0.1);}
            else if(m1)                        {sc.moveTo(0.25);}
            else if(m2)                        {sc.moveTo(0.75, 0.25);}
            else                               {sc.moveAt(spd);}

            // Display the current value
            String mpStr = sc.toString();
            dashboard.displayText(p++, mpStr);
            RobotLog.dd(TAG, mpStr);

            dashboard.displayText(p++,"Press Stop to end test.");
            dashboard.displayText(p++,"Level     up : Dpad up");
            dashboard.displayText(p++,"LevelRate dn : Dpad down");
            dashboard.displayText(p++,  "MoveTo       : Dpad left");
            dashboard.displayText(p++,  "MoveToRate   : Dpad right");
            dashboard.displayText(p++,  "MoveAtRate   : R_STICK_Y");
            dashboard.displayText(p++, sc.toString());

            sleep(CYCLE_MS);
        }

        dashboard.displayText(  1, "Done." );
    }
}
