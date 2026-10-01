package org.firstinspires.ftc.teamcode.test;

import android.util.SparseArray;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

import java.util.Locale;

/**
 * This OpMode steps n servos positions up and down based on D-Pad user inputs.
 * This code assumes a Servo configured with the name "testservo#".
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 */
@TeleOp(name = "TestContServo", group = "Test")
//@Disabled
public class ContServoTest extends InitLinearOpMode
{

    private static final double INCREMENT = 0.02;     // servo spd incr
    private static final int     CYCLE_MS =  20;       // period of each cycle
    private static final double   MAX_PWR =  1.0;     // Maximum servo pwr
    private static final double   MIN_PWR = -1.0;     // Minimum servo pwr
    private static final double  STOP_PWR =  0.0;
    private double pwr = STOP_PWR;

    PwmControl.PwmRange extRange = new PwmControl.PwmRange(1000, 2000);

    // Define class members
    private static final int MAX_SERVOS = 4;

    private static final String TAG = "SJH_SST";
    protected static final boolean VERBOSE = RobotConstants.logVerbose;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this);

        SparseArray<CRServo> servos = new SparseArray<>(MAX_SERVOS);

        TouchSensor maxstop = null;
        TouchSensor minstop = null;
        DigitalChannel maxStopDI = null;
        DigitalChannel minStopDI = null;


        int minTouchCnt = 0;
        int maxTouchCnt = 0;

        for(int m = 0; m < MAX_SERVOS; m++)
        {
            String servoName = "testservo" + m;
            CRServo srv;

            try
            {
                srv = hardwareMap.get(CRServo.class, servoName);
                srv.setPower(STOP_PWR);
                if(VERBOSE) { RobotLog.dd(TAG, "Found motor " + servoName);}
                servos.put(m, srv);
            }
            catch(Exception e)
            {
                if(VERBOSE) { RobotLog.ww(TAG, "Problem finding servo " + servoName + " " + e);}
            }
        }

        boolean success = false;
        try
        {
            maxstop = hardwareMap.get(TouchSensor.class, "maxstop");
            success = true;
        }
        catch(Exception ignored)
        {
        }
        try
        {
            maxStopDI = hardwareMap.get(DigitalChannel.class, "maxstop");
            maxStopDI.setMode(DigitalChannel.Mode.INPUT);
            success = true;
        }
        catch(Exception ignored)
        {
        }
        if(!success) if(VERBOSE) { RobotLog.ee(TAG, "No max sensor maxstop found");}
        else if(VERBOSE) { RobotLog.dd(TAG, "Found maxstop");}

        success = false;
        try
        {
            minstop = hardwareMap.get(TouchSensor.class, "minstop");
            if(VERBOSE) { RobotLog.dd(TAG, "Found minstop sensor ");}
            success = true;
        }
        catch(Exception ignored)
        {
        }
        try
        {
            minStopDI = hardwareMap.get(DigitalChannel.class, "minstop");
            minStopDI.setMode(DigitalChannel.Mode.INPUT);
            success = true;
        }
        catch(Exception ignored)
        {
        }

        if(!success) if(VERBOSE) { RobotLog.ee(TAG, "No min sensor minstop found");}
        else if(VERBOSE) { RobotLog.dd(TAG, "Found minstop");}

        // Wait for the start button
        dashboard.displayText(0, "Press Start to run servos.");
        while(!isStarted())
        {
            for(int m = 0; m < MAX_SERVOS; m++)
            {
                CRServo srv = servos.get(m);
                if(srv != null)
                    dashboard.displayText(m,
                        String.format(Locale.US, "SRV_%d %.2f", m, srv.getPower()));
            }
            if((minstop !=null && minstop.isPressed()) ||
                (minStopDI != null && !minStopDI.getState()))
            {
                if(VERBOSE) { RobotLog.dd(TAG, "Min stop hit");}
                dashboard.displayText(MAX_SERVOS+1, "MIN stop pressed");
            }
            if((maxstop !=null && maxstop.isPressed()) ||
                (maxStopDI != null && !maxStopDI.getState()))
            {
                if(VERBOSE) {  RobotLog.dd(TAG, "Max stop hit");}
                dashboard.displayText(MAX_SERVOS+2, "MAX stop pressed");
            }
            sleep(10);
        }
        waitForStart();

        while(opModeIsActive())
        {
            gpad1.update();
            boolean fwd        = gpad1.pressed(ManagedGamepad.Button.D_UP);
            boolean bak        = gpad1.pressed(ManagedGamepad.Button.D_DOWN);
            boolean newfwd     = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean newbak     = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean zeroize    = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean fullize    = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean incrSpd    = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean decrSpd    = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
            boolean extend     = gpad1.just_pressed(ManagedGamepad.Button.Y);
            boolean chngDir    = gpad1.just_pressed(ManagedGamepad.Button.X);
            boolean enable     = gpad1.just_pressed(ManagedGamepad.Button.A);
            boolean disable    = gpad1.just_pressed(ManagedGamepad.Button.B);

            if(incrSpd) pwr += INCREMENT; pwr = Math.min(MAX_PWR, pwr);
            if(decrSpd) pwr -= INCREMENT; pwr = Math.max(MIN_PWR, pwr);
            if(zeroize) pwr = STOP_PWR;
            if(fullize) pwr = MAX_PWR;
            // Display the current value
            dashboard.displayText(MAX_SERVOS,
                String.format(Locale.US, "Servo pos %4.2f %s %s", pwr, newfwd, fwd));
            for(int m = 0; m < MAX_SERVOS; m++)
            {
                CRServo srv = servos.get(m);
                if(srv == null) continue;
                CRServoImplEx srvEx = null;
                if (srv instanceof CRServoImplEx)
                {
                    srvEx = (CRServoImplEx) srv;
                }

                boolean minStopTriggered = false;
                boolean maxStopTriggered = false;

                if((minstop !=null && minstop.isPressed())||
                   (minStopDI != null && !minStopDI.getState()))
                {
                    minStopTriggered = true;
                    if(minTouchCnt == 0)
                    {
                        if(VERBOSE) { RobotLog.dd(TAG, "Min stop hit");}
                    }
                    dashboard.displayText(MAX_SERVOS+1, "MIN stop pressed");
                    minTouchCnt++;
                }
                else
                {
                    minTouchCnt = 0;
                }

                if((maxstop !=null && maxstop.isPressed())||
                    (maxStopDI != null && !maxStopDI.getState()))
                {
                    maxStopTriggered = true;
                    if(maxTouchCnt == 0)
                    {
                        if(VERBOSE) {  RobotLog.dd(TAG, "Max stop hit");}
                    }
                    dashboard.displayText(MAX_SERVOS+2, "MAX stop pressed");
                    maxTouchCnt++;
                }
                else
                {
                    maxTouchCnt = 0;
                }

                if(disable && srvEx != null) {srvEx.setPwmDisable(); continue;}
                if( enable && srvEx != null) {srvEx.setPwmEnable(); continue;}
                if(extend  && srvEx != null) {srvEx.setPwmRange(extRange);}

                if(chngDir)
                {
                    DcMotorSimple.Direction curdir = srv.getDirection();
                    DcMotorSimple.Direction newdir = DcMotorSimple.Direction.FORWARD;
                    if(curdir == DcMotorSimple.Direction.FORWARD)
                        newdir = DcMotorSimple.Direction.REVERSE;
                    srv.setDirection(newdir);
                }

                double tmpPwr;
                if(fwd)
                {
                    tmpPwr = pwr;
                    if(maxStopTriggered && tmpPwr > 0 ||
                       minStopTriggered && tmpPwr < 0) tmpPwr = 0.0;
                    srv.setPower(tmpPwr);
                    if(newfwd)
                    {
                        if(VERBOSE) {  RobotLog.dd(TAG,"Fwd just pressed.  Pwr=%4.2f", tmpPwr);}
                    }
                }
                else if(bak)
                {
                    tmpPwr = -pwr;
                    if(maxStopTriggered && tmpPwr > 0 ||
                        minStopTriggered && tmpPwr < 0) tmpPwr = 0.0;
                    srv.setPower(tmpPwr);
                    if(newbak)
                    {
                        if(VERBOSE) { RobotLog.dd(TAG,"Bak just pressed.  Pwr=%4.2f", -pwr);}
                    }
                }
                else srv.setPower(STOP_PWR);
            }

            dashboard.displayText(MAX_SERVOS + 1, "Press Stop to end test.");
            dashboard.displayText(MAX_SERVOS + 2, "Incr pwr : R_BUMP");
            dashboard.displayText(MAX_SERVOS + 3, "Decr pwr : L_BUMP");
            dashboard.displayText(MAX_SERVOS + 4, "Zero pwr : Dpad left");
            dashboard.displayText(MAX_SERVOS + 5, "Max  pwr : Dpad right");
            dashboard.displayText(MAX_SERVOS + 6, "Change Dir: : X");
            dashboard.displayText(MAX_SERVOS + 7, "Extend Range: : Y");
            dashboard.displayText(MAX_SERVOS + 8, "Disable PWM: : A");
            dashboard.displayText(MAX_SERVOS + 9, "Enable PWM: : B");

            sleep(CYCLE_MS);
        }
    }
}
