package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.wifi.RobotControllerAccessPointAssistant;

public class CamMount
{
  public CamMount(HardwareMap map) { this.hwMap = map; }

  public boolean init(){

    boolean success;

    try{
      camServo = hwMap.get(Servo.class, "camservo");
      success = true;
      setCamPos(RobotConstants.CAM_STOW);
      if(camServo instanceof ServoImplEx)
      {
        ServoImplEx sie = ((ServoImplEx) camServo);
        PwmControl.PwmRange rng = sie.getPwmRange();
        RobotLog.dd(TAG, "Camservo pwm range: %.2f to %.2f",
                         rng.usPulseLower, rng.usPulseUpper);
      }
    }catch (Exception e)
    {
      RobotLog.ee(TAG, "ERROR: Cam Servo missing");
      success = false;
    }
    return success;
  }

  public void setCamPos(double pos)
  {
    if(camServo == null) return;
    camServo.setPosition(pos);
  }

  private static final String TAG = "SJH_CAM";
  public Servo camServo;
  protected HardwareMap hwMap;
}
