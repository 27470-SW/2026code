package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.CommonUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

@SuppressWarnings("unused")
public class MotorComponent
{
  private static final String TAG = "SJH_MCO";

  public MotorComponent(String cfgName, HardwareMap map)
  {
    this.name = cfgName;
    this.hwMap = map;

    RobotLog.dd(TAG, "MotorComponent CTOR for %s", cfgName);
    levelOffsets = new double[MAX_LEVELS];
    for (int l = 0; l < MAX_LEVELS; ++l)
    {
      levelOffsets[l] = 0.0;
    }
  }

  public boolean init(Motors.MotorModel mtrModel, double extGear)
  {
    this.model = mtrModel;
    this.extGear = extGear;

    RobotLog.dd(TAG, "MotorComponent init for %s %s", name, mtrModel);

    boolean success = false;
    computeCpi();
    maxCps = model.getCpr() * (model.getRpm() / 60.0);
    maxSpd = (model.getRpm() / 60.0) * extGear;

    try
    {
      motor = hwMap.get(DcMotorEx.class, name);
      motor.setDirection(DcMotorSimple.Direction.FORWARD);
      motor.setPower(0);
      ((DcMotorEx) motor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      //((DcMotorEx)motor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      setMode(RUN_WITHOUT_ENCODER);

      mode = RUN_WITHOUT_ENCODER;
      prevMode = mode;
      RobotLog.dd(TAG, "model:%s cpi:%.2f maxcps:%.2f maxSpd:%.2f el_spd:%.2f",
                  model, cpi, maxCps, maxSpd, RobotConstants.EL_SPD);
      success = true;
    }
    catch (Exception e)
    {
      RobotLog.ee(TAG, "ERROR no " + name + " in hardware map");
    }

    init_common();

    return success;
  }

  private CRServoImplEx crsix;

  public boolean init(double maxSrvoIps)
  {
    //Note for CRServo - maxIps can get approx value by using spec unload max rot speed
    //and "gearing" between servo and effector.  This includes circumference of gears/pulleys.
    RobotLog.dd(TAG, "%s init %.2f servo", name, maxSrvoIps);
    maxIps = maxSrvoIps;
    boolean success = false;
    try
    {
      motor = hwMap.get(CRServo.class, name);
      motor.setPower(0);
      if (motor instanceof CRServo)
      {
        RobotLog.dd(TAG, "%s motor is a CRServo", name);
        CRServo crs = (CRServo) motor;
        if (crs instanceof CRServoImpl)
        {
          RobotLog.dd(TAG, "%s crservo is a CRServoImpl", name);
        }
        if (crs instanceof CRServoImplEx)
        {
          RobotLog.dd(TAG, "%s crservo is a CRServoImplEx", name);
          crsix = (CRServoImplEx) motor;

          PwmControl.PwmRange curRange = crsix.getPwmRange();
          RobotLog.dd(TAG, "Existing servo range for %s : %.2f %.2f",
                      name, curRange.usPulseLower, curRange.usPulseUpper);
          if (usePwmRangeSet)
          {
            PwmControl.PwmRange newRng =
              new PwmControl.PwmRange(1000, 2000);
            RobotLog.dd(TAG, "New servo range for %s : %.2f %.2f",
                        name, newRng.usPulseLower, newRng.usPulseUpper);
            crsix.setPwmRange(newRng);
          }
        }
      }

      RobotLog.dd(TAG, "Found CRservo %s", name);

      success = true;
    }
    catch (Exception e)
    {
      RobotLog.ee(TAG, "ERROR in init of " + name + " in hardware map" + e);
    }

    init_common();

    return success;
  }

  int testState = 0;
  double upTime = 0;
  double dnTime = 0;
  double upPos = -1;
  double dnPos = -1;
  boolean stateStarted = false;
  ElapsedTime cfgTimer = new ElapsedTime();

  public void resetTest()
  {
    testState = 0;
    upTime = 0;
    dnTime = 0;
    upPos = -1;
    dnPos = -1;
    stateStarted = false;
  }

  public void findCfg()
  {
    RobotLog.dd(TAG, "Starting findCfg");
    if (testState == 0)
    {
      if (!stateStarted)
      {
        levelRate = 1;
        movingToLevel = 0;
        moveAtRate(-1);
        stateStarted = true;
      }
      if (minStopTriggered)
      {
        testState++;
        RobotLog.dd(TAG, "Found bottom in findCfg - moving up");
        cfgTimer.reset();
        curLoc = 0.0;
        movingToLevel = 2;
        moveAtRate(1);
      }
    }
    else if (testState == 1)
    {
      if (maxStopTriggered)
      {
        testState++;
        upTime = cfgTimer.seconds();
        RobotLog.dd(TAG, "Found top in findCfg - moving down");
        upPos = curLoc;
        cfgTimer.reset();
        movingToLevel = 0;
        moveAtRate(-1);
      }
    }
    else if (testState == 2)
    {
      if (minStopTriggered)
      {
        testState++;
        dnTime = cfgTimer.seconds();
        RobotLog.dd(TAG, "Found bottom in findCfg - done");
        dnPos = curLoc;
        movingToLevel = -1;
        moveAtRate(0.0);
        RobotLog.dd(TAG, "ips %.2f upPos %.2f upTime %.2f dnPos %.2f dnTime %.2f",
                    maxIps, upPos, upTime, dnPos, dnTime);
      }
    }
  }

  private void init_common()
  {
    levelSensors = new DigitalChannel[MAX_LEVELS];
    touchSensors = new TouchSensor[MAX_LEVELS];
    clrRngSensors = new ColorRangeSensor[MAX_LEVELS];
    sensorTriggered = new boolean[MAX_LEVELS];
    prevSensorTriggered = new boolean[MAX_LEVELS];

    for (int l = 0; l < MAX_LEVELS; ++l)
    {
      try
      {
        String sensName = name + "L" + l;
        DigitalChannel d = hwMap.get(DigitalChannel.class, sensName);
        levelSensors[l] = d;
        d.setMode(DigitalChannel.Mode.INPUT);
        numLevelSensors++;
        RobotLog.dd(TAG, "Found digital sensor " + sensName);
      }
      catch (Exception ignored)
      {
      }

      try
      {
        String sensName = name + "L" + l;
        touchSensors[l] = hwMap.get(TouchSensor.class, sensName);
        numLevelSensors++;
        RobotLog.dd(TAG, "Found touch sensor " + sensName);
      }
      catch (Exception ignored)
      {
      }

      try
      {
        String sensName = name + "L" + l;
        clrRngSensors[l] = hwMap.get(ColorRangeSensor.class, sensName);
        numLevelSensors++;
        RobotLog.dd(TAG, "Found color sensor " + sensName);
      }
      catch (Exception ignored)
      {
      }
    }

    overrideHardstop = false;
    overrideSoftstop = true;

    for (int a = 0; a < MAX_ACTIONS; ++a)
    {
      try
      {
        String sensName = name + "A" + a;
        DigitalChannel d = hwMap.get(DigitalChannel.class, sensName);
        actionDigitalSensors.add(d);
        d.setMode(DigitalChannel.Mode.INPUT);
        numActionSensors++;
        RobotLog.dd(TAG, "Found digital action sensor " + sensName);
      }
      catch (Exception ignored)
      {
      }

      try
      {
        String sensName = name + "A" + a;
        actionTouchSensors.add(hwMap.get(TouchSensor.class, sensName));
        numActionSensors++;
        RobotLog.dd(TAG, "Found touch action sensor " + sensName);
      }
      catch (Exception ignored)
      {
      }

      try
      {
        String sensName = name + "A" + a;
        actionClrRngSensors.add(hwMap.get(ColorRangeSensor.class, sensName));
        numActionSensors++;
        RobotLog.dd(TAG, "Found color action sensor " + sensName);
      }
      catch (Exception ignored)
      {
      }
    }
  }

  public double getMaxSpeed()
  {
    return maxSpd;
  }

  private void computeCpi()
  {
    cpi = model.getCpr() / extGear;
  }

  private int updateCount=0;

  public void update()
  {
    if (motor == null) return;
//    if (motor instanceof DcMotorEx)
//    {
    if(motor != null) {
      curEnc = ((DcMotorEx) motor).getCurrentPosition();
      curSpd = ((DcMotorEx) motor).getVelocity();
      curLoc = curEnc / cpi;
    }

    updateCount++;
    if (updateCount==500){
      RobotLog.dd(TAG, name+" Update working good!"  );
    updateCount=0;
    }

//
//      if (waitingOnTimer && mtrTmr.seconds() > mtrTimeout)
//      {
//        motor.setPower(0.0);
//        setMode(prevMode);
//        waitingOnTimer = false;
//      }
//    }
//    else if (motor instanceof CRServo)
//    {
//      curSpd = cmdRate;
//      double now = srvTimer.seconds();
//      double dt = now - timePrev;
//      timePrev = now;
//      curLoc += dt * cmdRate * maxIps;
//    }
//
    for (int l = 0; l < numLevelSensors; ++l)
    {
      if (touchSensors[l] != null)
        sensorTriggered[l] = touchSensors[l].isPressed();
//
      if (sensorTriggered[l] != prevSensorTriggered[l])
      {
        RobotLog.dd(TAG, name + " Sensor State " + l + ":" + sensorTriggered[l] + "numlvlsensors:" + numLevelSensors);
      }
    }
//
    if (numLevelSensors > 0)
    {
      minStopTriggered = sensorTriggered[0];
      if(minStopTriggered){
        RobotLog.dd(TAG, name + " Min Stop Triggered!");
      }
    }



    if (movingToLevel >= 0 && movingToLevel < numLevelSensors)
    {
      if (sensorTriggered[movingToLevel] &&
          !prevSensorTriggered[movingToLevel])
      {
        RobotLog.dd(TAG, "Reached dest level %d levelPos %.2f curLoc %.2f",
                    movingToLevel, levelOffsets[movingToLevel], curLoc);
        movingToLevel = -1;
        moveAtRate(0.0);
      }
    }

    if (!overrideHardstop &&
        minStopTriggered && cmdRate < 0.0)
    {
      if (!prevSensorTriggered[0])
      {
        RobotLog.dd(TAG, "%s minstop stopping pwr rate:%.2f %.2f",
                    name, cmdRate, curLoc);
      }
      movingToLevel = -1;
      moveAtRate(0.0);


      curLoc = levelOffsets[0];
    }

//    for (int l = 0; l < numLevelSensors; ++l){
//
//      prevSensorTriggered[l] = sensorTriggered[l];
//
//    }

    if (!overrideSoftstop &&
        (curLoc >= softMax && cmdRate > 0.0 ||
         curLoc <= softMin && cmdRate < 0.0))
    {
      RobotLog.dd(TAG, "Softstop reached pwr rate:%.2f %s %s %.2f",
                  cmdRate, softMin, softMax, curLoc);
    }

    System.arraycopy(sensorTriggered, 0, prevSensorTriggered, 0, MAX_LEVELS);

//    actionTriggered =false;
//    for (DigitalChannel d : actionDigitalSensors)
//    {
//      if (!d.getState()) actionTriggered = true;
//    }
//    for (ColorRangeSensor c : actionClrRngSensors)
//    {
//      if (c.getDistance(DistanceUnit.INCH) < actionDist) actionTriggered = true;
//    }
//    for (TouchSensor t : actionTouchSensors)
//    {
//      if (t.isPressed()) actionTriggered = true;
//    }
//    if (actionTriggered && !prevActionTriggered)
//    {
//      RobotLog.dd(TAG, name + " Action triggered");
//      if (actionTriggered) numActionEvents++;
//
//      if(lockOnAction && !lock && !lockInDelay &&
//         CommonUtil.getInstance().getLinearOpMode().opModeIsActive())
//      {
//        RobotLog.dd(TAG,"%s Starting lock delay", name);
//        lockInDelay = true;
//        lockTmr.reset();
//      }
//    }
//    prevActionTriggered = actionTriggered;
//
//    if(lockOnAction && !lock && lockInDelay &&
//       lockTmr.seconds() > lockActionDelay &&
//       CommonUtil.getInstance().getLinearOpMode().opModeIsActive())
//    {
//      RobotLog.dd(TAG,"%s Starting lock duration - at %.2f", name, lockActionSpd);
//      moveAtRate(lockActionSpd);
//      lockInDelay = false;
//      lock = true;
//      lockTmr.reset();
//    }

//    if(lockOnAction && lock && lockTmr.seconds() > lockActionDuration &&
//       CommonUtil.getInstance().getLinearOpMode().opModeIsActive())
//    {
//      RobotLog.dd(TAG,"%s Unlocking setting rate to 0", name);
//      lock = false;
//      moveAtRate(0.0);
//    }
  }

  public String toString()
  {
    return String.format
      (Locale.US, "%s enc:%d cps:%.2f ips:%.2f spd:%.2f loc:%.2f %s %s %s %s",
       name, curEnc, curSpd, curSpd / cpi, curSpd / maxCps, curLoc,
       sensorTriggered[0], sensorTriggered[1], sensorTriggered[2], actionTriggered);
  }

  public DcMotor.RunMode getMode()
  {
    return mode;
  }

  public void setMode(DcMotor.RunMode mode)
  {
    if (this.mode == mode || motor == null || !(motor instanceof DcMotorEx))
      return;

    ((DcMotorEx) motor).setMode(mode);
    this.mode = mode;
  }

  public void setDir(DcMotorSimple.Direction dir)
  {
    if (motor == null) return;
    motor.setDirection(dir);
    curDir = dir;
  }

  public void setMaxSpeed(double maxSpd)
  {
    this.maxSpd = maxSpd;
  }

  public void setMaxIps(double maxIps)
  {
    this.maxIps = maxIps;
  }

  public void setSpeedMode()
  {
    setMode(RUN_USING_ENCODER);
  }

  public void setPowerMode()
  {
    setMode(RUN_WITHOUT_ENCODER);
  }

  public void setPidfCoeffs(double kP, double kI, double kD, double kF, DcMotor.RunMode mode)
  {
    if (motor == null || !(motor instanceof DcMotorEx)) return;

    if (mode == RUN_TO_POSITION)
    {
      ((DcMotorEx) motor).setPositionPIDFCoefficients(kP);
    }
    else
      ((DcMotorEx) motor).setVelocityPIDFCoefficients(kP, kI, kD, kF);
  }

  public void setLevelOffset(double... levelOffsets)
  {
    this.levelOffsets = levelOffsets;
    numSoftLevels = this.levelOffsets.length;
    softMin = levelOffsets[0];
    if (numSoftLevels > 1)
    {
      softMax = levelOffsets[numSoftLevels - 1];
    }
    for (double lev : levelOffsets)
    {
      if (lev < softMin) softMin = lev;
      if (lev > softMax) softMax = lev;
    }
    softRng = softMax - softMin;
    RobotLog.dd(TAG, name + " " + numSoftLevels + " softLevels set");
    RobotLog.dd(TAG, "softMin = %.2f", softMin);
    if (softMax < Double.MAX_VALUE)
    {
      RobotLog.dd(TAG, "softMax = %.2f softRng = %.2f", softMax, softRng);
    }
  }

  public void setDisableOnEndstop(boolean disableAtMin, boolean disableAtMax)
  {
    this.disableOnMinstop = disableAtMin;
    this.disableOnMaxstop = disableAtMax;
  }

  public void setLockOnAction(boolean lockOnAction)
  {
    this.lockOnAction = lockOnAction;
  }

  public void setLockActionDuration(double duration)
  {
    lockActionDuration = duration;
  }

  public void setLockActionDelay(double delay)
  {
    lockActionDelay = delay;
  }

  public void setLockActionSpd(double speed)
  {
    lockActionSpd = speed;
  }

  public void setActionDist(double dist) {actionDist  = dist;}


  public void startTimer(double timeout)
  {
    mtrTmr.reset();
    mtrTimeout = timeout;
    waitingOnTimer = true;
  }

  public void moveAtControlRate(double rate)
  {
    double wakeupThresh = 0.04;
    double absRate = Math.abs(rate);

    //This method is used for rates based on controller stick from driver/operator
    //If the motor can be sent to a position (moveToLevel, moveTo, etc),
    //we don't want 0 values from stick to stop the motor while moving to destination

    if (motor instanceof DcMotorEx && mode == RUN_TO_POSITION && absRate >= wakeupThresh)
    {
      //If in RUN_TO_POSITION and user calls this method, assume they
      //want to shift to prev motor mode
      RobotLog.dd(TAG, "Leaving RTP due to controller rate");
      if (prevMode != RUN_TO_POSITION)
      {
        setMode(prevMode);
      }
      else
      {
        setMode(RUN_USING_ENCODER);
      }
    }
    RobotLog.dd(TAG, "in moveatcontrolrate: movingtolevel: %d, absrate: %f, wakeupthresh: %f, rate: %f", movingToLevel, absRate, wakeupThresh, rate);
    if (movingToLevel < 0 || absRate >= wakeupThresh)
      moveAtRate(rate);
  }

  public void moveAtRate(double rate)
  {
    if (motor == null) return;

    if ((minStopTriggered && rate < 0) ||
        (maxStopTriggered && rate > 0))
    {
      //RobotLog.dd(TAG, "Forcing rate to 0 due to stops");
      cmdRate = 0.0;
      motor.setPower(cmdRate);
      return;
    }

    if(lock) return;

    if (motor instanceof DcMotorEx)
    {
      if (!overrideSoftstop &&
          (curLoc >= softMax && rate > 0.0 ||
           curLoc <= softMin && rate < 0.0))
      {
        cmdRate = 0.0;
        motor.setPower(cmdRate);
        return;
      }

      if (cmdRate != rate)
      {
        cmdRate = rate;
        if (mode == RUN_USING_ENCODER)
        {
          //          ((DcMotorEx) motor).setVelocity(cmdRate * maxCps);
          motor.setPower(cmdRate);
        }
        else
        {
          motor.setPower(cmdRate);
        }
      }
      RobotLog.dd(TAG, "command rate = %f", cmdRate);
    }
    else if (motor instanceof CRServo)
    {
      if (usePwmDisable && pwmDisabled && cmdRate != 0.0 && (crsix != null))
      {
        RobotLog.dd(TAG, "%s Enabling PWM", name);
        crsix.setPwmEnable();
        pwmDisabled = false;
      }

      if (cmdRate != rate)
      {
        cmdRate = rate;
        RobotLog.dd(TAG, "%s Calling setPower cmdRate:%.2f", name, cmdRate );
        motor.setPower(cmdRate);
      }
    }
  }

  public void moveToCnt(int cnt, double pwr)
  {
    if (motor == null) return;
    RobotLog.dd(TAG, "%s moveToCnt %d at pwr %.2f", name, cnt, pwr);
    if (motor instanceof DcMotorEx)
    {
      DcMotor.RunMode tmpMode = ((DcMotorEx) motor).getMode();
      if (tmpMode != RUN_TO_POSITION) prevMode = tmpMode;
      ((DcMotorEx) motor).setTargetPosition(cnt);
      setMode(RUN_TO_POSITION);
      motor.setPower(pwr);
      cmdRate = pwr;
    }
  }

  public void moveTo(double offset, double pwr)
  {
    if (motor == null) return;
    RobotLog.dd(TAG, "%s moveTo %.2f at pwr %.2f", name, offset, pwr);
    double destLoc = Range.clip(offset, softMin, softMax);
    if (motor instanceof DcMotorEx)
    {
      DcMotor.RunMode tmpMode = ((DcMotorEx) motor).getMode();
      if (tmpMode != RUN_TO_POSITION) prevMode = tmpMode;
      int destCnt = (int) (destLoc * cpi);
      RobotLog.dd(TAG, "%s moveTo RTP dLoc %.2f dCnt %d pwr %.2f",
                  name, destLoc, destCnt, pwr);
      ((DcMotorEx) motor).setTargetPosition(destCnt);
      setMode(RUN_TO_POSITION);
      motor.setPower(pwr);
      cmdRate = pwr;
    }
    if (motor instanceof CRServo)
    {
      //Not implemented for CRServo due to lack of position feedback or
      //other means to accurately interpolate position
      RobotLog.ee(TAG, "Not implemented for CRServo");
    }
  }

  public void moveToLevel(int level, double pwr)
  {

    if (numSoftLevels == 0 ||
        level > numSoftLevels ||
        (level >= numLevelSensors && motor instanceof CRServo))
    {
      RobotLog.ee(TAG, "Not enough levels %d %d", level, numSoftLevels);
      return;
    }
    RobotLog.dd(TAG, "%s In moveToLevel level=%d pwr=%.2f softLevs=%d levSens=%d",
          name, level, pwr, numSoftLevels, numLevelSensors);
    RobotLog.dd(TAG, "curLoc: %.2f level destLoc: %.2f",
            curLoc, levelOffsets[level]);

    if (motor instanceof DcMotorEx)
    {
      moveTo(levelOffsets[level], pwr);
    }
    else if (motor instanceof CRServo)
    {
      movingToLevel = -1;
      //Due to lack of position feedback with CRServo, always use positive power.
      //This assumes direction is set such that +pwr moves away from minstop
      levelRate = Math.abs(pwr);
      movingToLevel = level;
      if (maxStopTriggered) levelRate = -levelRate;
      if ((minStopTriggered && level == 0) ||
          (maxStopTriggered && numLevelSensors > 1 && level == numLevelSensors - 1))
      {
        levelRate = 0.0;
        movingToLevel = -1;
      }

      if (!minStopTriggered && level == 0)
        levelRate = -levelRate;

      moveAtRate(levelRate);
    }
  }

  public void moveDist(double dist, double pwr)
  {
    if (motor == null) return;
    moveTo(curLoc + dist, pwr);
  }

  public void moveToPct(double pct, double pwr)
  {
    if (motor == null) return;
    double dstLoc = pct * softRng + softMin;
    moveTo(dstLoc, pwr);
  }

  public void movePct(double pct, double pwr)
  {
    moveDist(pct * softRng, pwr);
  }

  public void stop()
  {
    moveAtRate(0.0);
  }

  public int getCurEnc()
  {
    return curEnc;
  }

  public double getCurSpd()
  {
    return curSpd;
  }

  public double getPos()
  {
    return curEnc / cpi;
  }

  public double getPwr()
  {
    return cmdRate;
  }

  public boolean[] getSensorTriggered()
  {
    return sensorTriggered;
  }

  public boolean getActionTriggered()
  {
    return actionTriggered;
  }
  public int getNumActionEvents()
  {
    return numActionEvents;
  }

  public boolean getMovingToLevel()
  {
    if (motor == null) return false;
    if(motor instanceof DcMotorEx)
    {
      return ((DcMotorEx) motor).getMode() == RUN_TO_POSITION && ((DcMotorEx) motor).isBusy();
    }
    else
    {
      return movingToLevel > -1;
    }
  }

  public DcMotorSimple.Direction getCurDir()
  {
    return curDir;
  }

  protected HardwareMap hwMap;
  private DcMotorSimple motor = null;

  private final String name;
  private int curEnc = 0;
  private double curSpd = 0.0;
  private double curLoc = 0.0;
  private double cmdRate = 0.0;
  DcMotorSimple.Direction curDir = DcMotorSimple.Direction.FORWARD;
  //gearing between motor gearbox output shaft and end movement - include wheel & pulley diams
  //effector movement per output shaft revolution
  //this will be in in/rev for wheels and pulleys
  //this will be in deg/rev for things like rotating arms that have linkage or cams
  public double extGear = 1.0;
  private double cpi; // encoder counts per inch
  private double maxIps = 0.0; //Max inches per second
  private Motors.MotorModel model = Motors.MotorModel.GOBILDA_5202_19_2;
  private DcMotor.RunMode mode;
  DcMotor.RunMode prevMode = mode;
  double maxCps = model.getCpr() * (model.getRpm() / 60.0);
  private double maxSpd  = (model.getRpm() /60.0) * extGear;

  private static final int MAX_ACTIONS = 10;
  private final List<DigitalChannel> actionDigitalSensors = new ArrayList<>();
  private final List<TouchSensor> actionTouchSensors = new ArrayList<>();
  private final List<ColorRangeSensor> actionClrRngSensors = new ArrayList<>();
  private int numActionSensors = 0;
  private boolean actionTriggered = false;
  private int numActionEvents = 0;
  private boolean prevActionTriggered = false;
  private boolean lockOnAction = true;
  private boolean lock = false;
  private boolean lockInDelay = false;
  private final ElapsedTime lockTmr = new ElapsedTime();
  private double lockActionDuration = 0.5;
  private double lockActionSpd = 0.0;
  private double lockActionDelay = 0.0;
  private double actionDist = 0.0;

  private double softMin = -Double.MAX_VALUE;
  private double softMax = Double.MAX_VALUE;
  private double softRng = softMax - softMin;
  private final int MAX_LEVELS = 10;
  private DigitalChannel[] levelSensors;
  private TouchSensor[] touchSensors;
  private ColorRangeSensor[] clrRngSensors;
  private int numLevelSensors = 0;
  private boolean[] sensorTriggered;
  private boolean[] prevSensorTriggered;
  private boolean minStopTriggered = false;
  private boolean maxStopTriggered = false;
  private boolean overrideHardstop = false;
  private boolean overrideSoftstop = true;
  private boolean disableOnMinstop = true;
  private boolean disableOnMaxstop = true;
  private static final boolean usePwmRangeSet = false;
  private static final boolean usePwmDisable = true;
  private static boolean pwmDisabled = false;
  private final double pwmRevTime = 0.2;
  private final ElapsedTime pwmTmr = new ElapsedTime();
  private final ElapsedTime mtrTmr = new ElapsedTime();
  private double mtrTimeout = -1.0;
  private boolean waitingOnTimer = false;

  private int movingToLevel = -1;
  private double levelRate = 0.0;

  private double[] levelOffsets;
  private int numSoftLevels = 0;

  private final ElapsedTime srvTimer = new ElapsedTime();
  private double timePrev = srvTimer.startTime();
  private static final double servoThresh = 0.5;
}