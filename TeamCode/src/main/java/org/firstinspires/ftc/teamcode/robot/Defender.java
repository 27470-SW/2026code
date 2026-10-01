package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class Defender
{
  public Defender(HardwareMap map) { this.hwMap = map; }

  public boolean init(){

    boolean success;

    try{
      defendServo = hwMap.get(Servo.class, "Defender");
      success = true;
      setDefendPos(RobotConstants.DefendStow);
    }catch (Exception e)
    {
      RobotLog.ee(TAG, "ERROR: Defend Servo missing");
      success = false;
    }
    return success;
  }

  public void setDefendPos(double pos)
  {
    if (defendServo == null) return;
    defendServo.setPosition(pos);
  }

  private static final String TAG = "SJH_DEF";
  public Servo defendServo;
  protected HardwareMap hwMap;
}
