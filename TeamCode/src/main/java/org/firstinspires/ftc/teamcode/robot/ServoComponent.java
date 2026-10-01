package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

@SuppressWarnings("unused")
public class ServoComponent
{
  private static final String TAG = "SJH_SCO";

  public ServoComponent(String cfgName, HardwareMap map)
  {
    this.name = cfgName;
    this.hwMap = map;
    levelOffsets = new double[MAX_LEVELS];
    for(int l=0; l < MAX_LEVELS; ++l)
    {
      levelOffsets[l] = 0.0;
    }
  }

  public boolean init(double min, double max){
    minPos = min;
    maxPos = max;
    return init();
  }
  public boolean init()
  {
    boolean success = false;

    try
    {
      servo = hwMap.get(Servo.class, name);
      servo.setDirection(Servo.Direction.FORWARD);
      RobotLog.dd(TAG, "Found Servo device " + name);
      success = true;
    }
    catch(Exception e)
    {
      RobotLog.ee(TAG, "No Servo device found " + name);
    }
    return success;
  }

  public void setExtended()
  {
    if(servo == null) return;
    if(servo.getController() instanceof ServoControllerEx)
    {
      Servo s = servo;
      ServoControllerEx srvoCntlr = (ServoControllerEx) (s.getController());

      int port = s.getPortNumber();
      PwmControl.PwmRange curRange = srvoCntlr.getServoPwmRange(port);
      RobotLog.dd(TAG, "Existing servo range for %s : %.2f %.2f",
          name, curRange.usPulseLower, curRange.usPulseUpper);

      PwmControl.PwmRange newRng =
          new PwmControl.PwmRange(500, 2500);
      RobotLog.dd(TAG, "New servo range for %s : %.2f %.2f",
          name, newRng.usPulseLower, newRng.usPulseUpper);
      srvoCntlr.setServoPwmRange(port, newRng);
    }
  }

  public void setDirection(Servo.Direction dir)
  {
    if(servo == null) return;
    servo.setDirection(dir);
    cmdDir = dir;
  }

  public void setLevelOffset(double... levelOffsets)
  {
    this.levelOffsets = levelOffsets;
    numSoftLevels = this.levelOffsets.length;
  }

  public void scaleRange(double min, double max)
  {
    if(servo == null) return;
    if(min > max) servo.scaleRange(max, min);
    else servo.scaleRange(min, max);
  }

  public void moveTo(double pos)
  {
    if(servo == null) return;
    if(cmdPos != pos)
    {
      if (pos > maxPos){pos = maxPos;}
      if (pos < minPos){pos = minPos;}
      servo.setPosition(pos);
      cmdPos = pos;
    }
  }

  public void moveTo(double pos, double rate)
  {
    if(cmdPos == pos) return;
    if(pos > cmdPos) cmdRate = Math.abs(rate);
    else             cmdRate = -Math.abs(rate);
    destPos = pos;
    if(!movingToTarget) prevTime = tmr.seconds();
    movingToTarget = true;
  }

  public void moveAmount(double amt){
    if(servo == null) return;
    double currentPos = servo.getPosition();

    double pos = currentPos +amt;

    moveTo(pos);
    cmdPos = pos;

  }

  public void moveAt(double rate)
  {
    cmdRate = rate;
  }

  public void moveToLevel(int level)
  {
    if (numSoftLevels > level) moveTo(levelOffsets[level]);
  }

  public void moveToLevel(int level, double rate)
  {
    if (numSoftLevels > level) moveTo(levelOffsets[level], rate);
  }

  public void makeDeadServo(){
    servo.getController().pwmDisable();
  }

  public void update()
  {
    double now = tmr.seconds();
    double dt = now - prevTime;
    prevTime = now;
    double deltaPos = dt * cmdRate;
    double pos = Range.clip(cmdPos + deltaPos, 0, 1);

    if(movingToTarget)
    {
      if(destPos > cmdPos  && pos >=  destPos ||
          destPos < cmdPos  && pos <=  destPos ||
          destPos == cmdPos)
      {
        pos = destPos;
        movingToTarget = false;
        cmdRate = 0.0;
      }
      moveTo(pos);
    }
    else
    {
      moveTo(pos);
    }
  }

  public String toString()
  {
    return String.format(Locale.US, "%s spd:%.2f loc:%.2f dir:%s, mvg:%s",
        name, cmdRate, cmdPos, cmdDir, movingToTarget);
  }

  public boolean isMovingToTarget()
  {
      return  movingToTarget;
  }
  public void setMovingToTarget(boolean target)
  {
    movingToTarget = target;
  }
  public double getPosition(){return servo.getPosition();}


  private final String name;
  private final HardwareMap hwMap;
  private Servo servo = null;

  private final ElapsedTime tmr = new ElapsedTime();
  double delayTime = 0.02;
  double prevTime = tmr.seconds();

  private double cmdPos = -2.0;
  private double cmdRate = 0.0;
  private Servo.Direction cmdDir;
  private double destPos;
  private boolean movingToTarget = false;

  private double[] levelOffsets;
  private static final int MAX_LEVELS = 10;
  private int numSoftLevels = 0;
  private double minPos = 0;
  private  double maxPos = 1;
}
