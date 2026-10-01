package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

public class Spinner
{
    public Spinner(HardwareMap map)
    {
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            spinnerR = hwMap.get(DcMotorEx.class, "spinner");
            spinnerR.setDirection(RobotConstants.SP_RED_DIR);
            spinnerR.setPower(0);
            spinnerR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            spinnerR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinnerR.setMode(RUN_WITHOUT_ENCODER);

            mode = RUN_WITHOUT_ENCODER;
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initSpinner\n" + e.toString());
        }

        try
        {
            spinnerB = hwMap.get(DcMotorEx.class, "spinnerb");
            spinnerB.setDirection(RobotConstants.SP_RED_DIR);
            spinnerB.setPower(0);
            spinnerB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            spinnerB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinnerB.setMode(RUN_WITHOUT_ENCODER);

            mode = RUN_WITHOUT_ENCODER;
            success = true;
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR get hardware map initSpinner\n" + e.toString());
        }

        return success;
    }

    public void update()
    {
        if(spinnerR != null)
        {
            encPos = spinnerR.getCurrentPosition();
            curSpd = spinnerR.getVelocity();
        }
    }

    public String toString()
    {
        return String.format(Locale.US, "spin %5d %4.2f", encPos, curSpd);
    }

    public void stop()
    {
        if(spinnerR != null) spinnerR.setPower(0.0);
        if(spinnerB != null) spinnerB.setPower(0.0);
    }

    public void setMode(DcMotor.RunMode mode)
    {
        if(this.mode != mode && spinnerR != null)
        {
            spinnerR.setMode(mode);
        }
        if(this.mode != mode && spinnerB != null)
        {
            spinnerB.setMode(mode);
        }
        this.mode = mode;
    }

    public void setPower(double pwr)
    {
        if(spinnerR != null && cmdPwr != pwr) spinnerR.setPower(pwr);
        if(spinnerB != null && cmdPwr != pwr) spinnerB.setPower(pwr);
        cmdPwr = pwr;
    }

    public void setDir(DcMotorSimple.Direction dir)
    {
        spinnerR.setDirection(dir);
        spinnerB.setDirection(dir);
    }

    protected HardwareMap hwMap;
    private DcMotorEx spinnerR = null;
    private DcMotorEx spinnerB = null;
    private static final String TAG = "SJH_SPN";

    private int encPos = 0;
    private double curSpd = 0.0;

    private double cmdPwr = 0.0;

    private DcMotor.RunMode mode = RUN_USING_ENCODER;
}