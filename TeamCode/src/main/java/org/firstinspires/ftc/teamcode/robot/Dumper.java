package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

public class Dumper
{
    private static final String TAG = "SJH_DMP";

    public Dumper(HardwareMap map)
    {
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = true;
        try
        {
            dumpServo = hwMap.get(Servo.class, "dumper");

            if(RobotConstants.DP_DUMP_STOW > RobotConstants.DP_DUMP_DEPLOY)
            {
                dumpServo.setDirection(Servo.Direction.REVERSE);
                dumpServo.scaleRange(RobotConstants.DP_DUMP_DEPLOY, RobotConstants.DP_DUMP_STOW);
                RobotLog.dd(TAG, "Scaling dumper from %.2f-%.2f to 0-1 dir:REV",
                            RobotConstants.DP_DUMP_DEPLOY, RobotConstants.DP_DUMP_STOW);
            }
            else
            {
                dumpServo.setDirection(Servo.Direction.FORWARD);
                dumpServo.scaleRange(RobotConstants.DP_DUMP_STOW, RobotConstants.DP_DUMP_DEPLOY);
                RobotLog.dd(TAG, "Scaling dumper from %.2f-%.2f to 0-1 dir:FWD",
                            RobotConstants.DP_DUMP_STOW, RobotConstants.DP_DUMP_DEPLOY);
            }

            stow();
        }
        catch (Exception e)
        {
            RobotLog.ww(TAG, "ERROR dumper - no dumpServo dumper\n" + e.toString());
            success = false;
        }

        return success;
    }

    public void update()
    {
        if(isMoving)
        {
            double now = mvTmr.seconds();
            double dt = now - mvTime;
            mvTime = now;
            double newLoc = Range.clip(dt * rate + dmpLoc, 0.0, 1.0);

            if(dstLoc > srcLoc  && newLoc >=  dstLoc ||
               dstLoc < srcLoc  && newLoc <=  dstLoc ||
               dstLoc == srcLoc)
            {
                newLoc = dstLoc;
                isMoving = false;
                rate = 0.0;
            }

            RobotLog.dd(TAG,"Dumper strt:%.2f dst:%.2f loc:%.2f dt:%.2f rate:%.2f",
                        srcLoc, dstLoc, newLoc, dt, rate);

            setDumpPos(newLoc);
        }
    }

    public void setDumpPos(double pos)
    {
        if(pos != dmpLoc && dumpServo != null)
        {
            dmpLoc = pos;
            dumpServo.setPosition(dmpLoc);
        }
    }

    public void moveToPosAtRate(double pos, double pctPerSecond)
    {
        RobotLog.dd(TAG,"Dumper moveToPosAtRate pos:%.2f rate:%.2f",
                    pos, pctPerSecond);
        srcLoc = dmpLoc;
        dstLoc = pos;
        rate   = Math.abs(pctPerSecond);
        double deltaLoc = dstLoc - srcLoc;
        isMoving = false;
        if(deltaLoc != 0)
        {
            isMoving = true;
            if(deltaLoc < 0) rate = -rate;
            mvTmr.reset();
            mvTime = mvTmr.seconds();
        }
        else
        {
            rate = 0.0;
        }
    }

    public void stow()   { setDumpPos(0.0); }
    public void deploy() { setDumpPos(1.0); }

    public void adjDumpPos(double incr)
    {
        setDumpPos(dmpLoc + incr);
    }

    @SuppressWarnings("unused")
    public void toggleDumpPos ()
    {
        if (dmpLoc < 0.5) setDumpPos(0.6);
        else              setDumpPos(0.0);
    }

    public String toString()
    {
        return String.format(Locale.US,
                "dumper %4.2f", dmpLoc);
    }

    private Servo dumpServo;
    protected HardwareMap hwMap;

    private double dmpLoc = -2.0;
    private double srcLoc = -2.0;
    private double dstLoc = -2.0;
    private double rate = 0.0;
    private final ElapsedTime mvTmr = new ElapsedTime();
    private double mvTime;
    private boolean isMoving = false;
}
