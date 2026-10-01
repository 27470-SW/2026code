package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

public class Loader
{
    public Loader(String cfgName, HardwareMap map)
    {
        this.name  = cfgName;
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            loadMotor = hwMap.get(DcMotorEx.class, name);
            loadMotor.setDirection(RobotConstants.LD_DIR);
            loadMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            loadMotor.setPower(0.0);
            curLdrMotorPwr = 0.0;

            success = true;
        }
        catch (Exception e)
        {
          RobotLog.ee(TAG, "ERROR in loader init\n" + e.toString());
        }

        return success;
    }

    public void update()
    {
        if(loadMotor != null)
        {
            encPos = loadMotor.getCurrentPosition();
            curSpd = loadMotor.getVelocity();
        }
    }

    public void setPower(double pwr)
    {
        if(loadMotor != null && pwr != curLdrMotorPwr)
        {
            loadMotor.setPower(pwr);
            curLdrMotorPwr = pwr;
        }
    }

    public String toString(){
        return String.format(Locale.US, "shoot %5d %4.2f",
                encPos, curSpd);
    }

    private double curLdrMotorPwr;
    private DcMotorEx loadMotor;
    private Servo loadServo;
    protected HardwareMap hwMap;
    protected final String name;

    private static final String TAG = "SJH_LDR";
    private int encPos;
    private double curSpd;
}
