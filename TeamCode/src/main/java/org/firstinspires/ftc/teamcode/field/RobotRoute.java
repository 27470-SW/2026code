package org.firstinspires.ftc.teamcode.field;

import static android.os.SystemClock.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.Decode_Auton;
import org.firstinspires.ftc.teamcode.robot.MecanumBot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.HalDashboard;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import static org.firstinspires.ftc.teamcode.field.Field.Alliance.RED;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.robot.Shooter.BALL_CHOICE.*;

/* ==============================================================================================
 * RobotRoute: what each route function does ON THE ROBOT.
 *
 * The routes themselves (Decode_Route, MAINFARROUTERED, ...) and the poses (in Decode_Route) are
 * in SharedRoutes, shared with MeepMeep.  This file is TeamCode only, so it can use the robot.
 * MeepMeepTesting has its own version of these functions in MeepMeepRoute.
 *
 * A function that routes call must also be listed (abstract) in Decode_Route.
 * Helpers that only these functions use (moveArmToDrop, openclaw, ...) can just live here.
 * ============================================================================================== */
@SuppressWarnings("unused")
public class RobotRoute extends Decode_Route
{
    private static final String TAG = "SJH_RTE";

    public RobotRoute(MecanumBot robot,
                      PositionOption startPos,
                      Field.Park_Pos parkPos,
                      Field.Wiffle_Pos lastLocation,
                      Field.Motif motif,
                      Field.Num_shots numshot)
    {
        super(startPos, parkPos, lastLocation, motif, numshot);   // builds the route
        this.robot = robot;
        goalLocation = alliance==RED?RED_GOAL_POSE:BLUE_GOAL_POSE;
    }

    protected MecanumBot robot;

    /** Set by the opmode before running, so conditions can use the camera, match timer, etc. */
    public Decode_Auton opMode;

    public final static int INIT_CONE_STACK = 5;
    public static int conestackNum = INIT_CONE_STACK;

//	 public void elvToConeStack()
//     {
//         switch (conestackNum)
//         {
//              case 1:
//                  liftElvConeStackLvlOne();
//                  break;
//              case 2:
//                  liftElvConeStackLvlTwo();
//                  break;
//              case 3:
//                  liftElvConeStackLvlThree();
//                  break;
//              case 4:
//                  liftElvConeStackLvlFour();
//                  break;
//              case 5:
//                  liftElvConeStackLvlFive();
//                  break;
//              default:
//                  liftElvConeStackLvlOne();
//                  break;
//         }
//         conestackNum --;
//     }
//
//    protected void liftElvGndJnct()
//    {
//        if (robot.elev != null)
//        {
//            robot.elev.moveToLevel(0, RobotConstants.EL_SPD);
//        }
//
//    }
//
//     protected void liftElvLowPoleJnct()
//     {
//         if (robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving Lifter to %.2f",
//                         RobotConstants.LF_EXT_LEVS[0] + 1.0);
//             RobotLog.dd(TAG, robot.elev.toString());
//
//             robot.elev.moveToLevel(1, RobotConstants.EL_SPD);
//         }
//     }
//
//     protected void liftElvMediumPoleJnct()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elev at +rate");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(2, RobotConstants.EL_SPD);
//         }
//
//     }
//
//     /* Lift Elevator to the Highest Level to prepare to drop Cone */
//     protected void liftElvHighPoleJnct()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving armExtend to level 1");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(3, RobotConstants.EL_SPD);
//         }
//     }
//
//     /* Lift Elevator to the Highest Level to prepare to drop Cone */
//     protected void liftElvConeStackLvlFive()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elevator to cone stack level 5");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(4, RobotConstants.EL_SPD);
//         }
//     }
//
//     protected void liftElvConeStackLvlFour()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elevator to cone stack level 4");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(5, RobotConstants.EL_SPD);
//         }
//     }
//
//     protected void liftElvConeStackLvlThree()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elevator to cone stack level 3");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(6, RobotConstants.EL_SPD);
//         }
//     }
//
//     protected void liftElvConeStackLvlTwo()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elevator to cone stack level 2");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(7, RobotConstants.EL_SPD);
//         }
//     }
//
//     protected void liftElvConeStackLvlOne()
//     {
//         if(robot.elev != null)
//         {
//             RobotLog.dd(TAG,"Moving elevator to cone stack level 1");
//             RobotLog.dd(TAG, robot.elev.toString());
//             robot.elev.moveToLevel(0, RobotConstants.EL_SPD);
//         }
//     }
//
    protected void setClawPos(double pos){
         
     }


     public void openclaw (){

         if(robot.claw != null)
         {
             robot.claw.openClaw(1);

         }


     }
//     protected void elvConeStackSecondOption()
//	 {
//         //setClawPos(.6);
//         liftElvGndJnct();
//         //setClawPos(.9);
//         liftElvMediumPoleJnct();
//    }
//
//     /* Release grasped Cone */
//     protected void openClaw()
//     {
////         if(robot.claw != null)
////         {
////             robot.claw.openClaw();
////         }
//     }
//
public void slidesUpOne(){
    RobotLog.dd(TAG, "set slides pos to 4");
    robot.slides.moveToLevel(4);
}



public void moveToPosition4(){
    slidesUpOne();
    armUpLittle();
}


    public void moveArmToBack(){
        RobotLog.dd(TAG, "moveArmToBack");
        robot.arm.moveToLevel(6, 1);
        robot.slides.moveToLevel(3);
    }

     protected void moveArmForward(){
         RobotLog.dd(TAG, "MOVE ARM FORWARD");
         robot.arm.moveToLevel(0,1);

     }

     protected void moveArmBackward(){
         RobotLog.dd(TAG, "MOVE Arm Back");
         robot.arm.moveToLevel(2,1);

     }

    public void armUpLittle(){
        RobotLog.dd(TAG, "Arm up pos little");
        robot.arm.moveToLevel(4, 1);

    }

    private Timer mTimer1;
    private TimerTask mTt1;
    private Timer mTimer2;
    private TimerTask mTt2;
    private Timer mTimer3;
    private TimerTask mTt3;


    @Override
    public void deliverSample() {
        RobotLog.dd(TAG, "Arm up pos little");
        moveArmToDrop();


        mTimer1 = new Timer();
        mTt1 = new TimerTask() {
            public void run() {
                openclaw();
            }
        };
        mTimer1.schedule(mTt1, 300);

        mTimer2 = new Timer();
        mTt2 = new TimerTask() {
            public void run() {
                moveArmTo90();
            }

        };
        mTimer2.schedule(mTt2, 700);

        mTimer3 = new Timer();
        mTt3 = new TimerTask() {
            public void run() {
                minSlides();
            }
        };
        mTimer3.schedule(mTt3, 850);
//        route.addFunction(route::moveArmToDrop );
//        route.addEvent(Route.Action.WAIT,0.2);
//        route.addFunction(route::openclaw);
//        route.addEvent(Route.Action.WAIT,0.3);
//        route.addFunction(route::moveArmTo90 );
//        route.addEvent(Route.Action.WAIT,0.2);
//        route.addFunction(route::minSlides);
  }
     protected void shootWiffleFar(){
         RobotLog.dd(TAG, "Shoot Balls Far from small triangle");
     }
    @Override
    protected void shootFar(){
        shootingFar = true;
        robot.shooter.defaultFarShooterTraj();
        robot.shooter.controlShooterW.setWheelVelocity(1500);
         RobotLog.dd(TAG, "Turn on wheel");
    }
    protected void shootFar3(){
        shootingFar = true;
        robot.shooter.defaultFarShooterTraj();
        robot.shooter.controlShooterW.setWheelVelocity(1750);
        RobotLog.dd(TAG, "Turn on wheel");
    }
    Pose2d goalLocation;
    double distanceToTheGoal;

    private double calcDistanceToGoal(){
        double x = goalLocation.getX() - robot.getPoseEstimate().getX();
        double y = goalLocation.getY() - robot.getPoseEstimate().getY();

        return Math.sqrt(x*x + y*y);
    }
    protected void autoTrajShooter_wheel(){
        distanceToTheGoal = calcDistanceToGoal();
        robot.shooter.spinShooterW(distanceToTheGoal);
        robot.shooter.onAutoTraj();

    }
    @Override
    protected void wheelOff(){
        robot.shooter.stopWheel();
    }
     @Override
     protected void shootWiffleClose(){
         robot.shooter.autonShootClose();
         robot.shooter.controlShooterW.setWheelVelocity(1225);
         RobotLog.dd(TAG, "Shoot Balls Close");
     }
     @Override
     protected void transitonUp(){
         RobotLog.dd(TAG, "Transition Up ALL");
     }

     @Override
     protected void intakeonandthreeTransitionsDown(){
         robot.crAzYIntake.setPwr(1);
         robot.shooter.resetTransition();
         robot.shooter.setShooterTrajPos(DEFAULT_TRAJ_POSITION);
    }
     protected void resetTrajAng(){
        robot.shooter.setShooterTrajPos(MIN_TRAJ_ENCODER);
     }
     @Override
     protected void intakeOn(){
         robot.crAzYIntake.setPwr(1);
        RobotLog.dd(TAG, "IntakeOn");
     }
     @Override
     protected void intakeOff (){
         robot.crAzYIntake.setPwr(0);
        RobotLog.dd(TAG, "Intake Off");
     }
     protected void wheel1350(){
         robot.shooter.controlShooterW.setWheelVelocity(1350);
     }
     protected void doPark()
     {
         if(VERBOSE) { RobotLog.dd(TAG, "Parking bot"); }
     }
    @Override
    protected void shoot1(){

        robot.shooter.shoot(RIGHT,true);

        RobotLog.dd(TAG, "Shoot Transition 1");}
    @Override
    protected void shoot2(){
        robot.shooter.shoot(CENTER,true);
        RobotLog.dd(TAG, "Shoot Transition 2");}
    @Override
    protected void shoot3(){
        robot.shooter.shoot(LEFT,true);
        RobotLog.dd(TAG, "Shoot Transition 3");}


    public boolean shootingFar = false;

    private double DRIVE_TO_SHOOT_AMT = 3;  //Was 5.6

    private Timer mTimerIntake;
    private TimerTask mTtIntake;

    @Override
    protected void shootAllThree(){
        shoot1();
        shoot2();
        shoot3();

//        mTimerIntake = new Timer();
//        mTtIntake = new TimerTask() {
//            public void run() {
//                intakeonandthreeTransitionsDown();
//                wheelOff();
//            }
//        };
//
//        mTimerIntake.schedule(mTtIntake, 1000);
    }

    private Timer mTimerShoot3;
    private TimerTask mTtShoot3;

    @Override
    protected void shoot3Wiffles(){
        intakeOff();
        shootFar3();

        mTimerShoot3 = new Timer();
        mTtShoot3 = new TimerTask() {
            public void run() {
                shootAllThree();
            }
        };

        mTimerShoot3.schedule(mTtShoot3, 2000);
    }

    @Override
    public void shoot3Wifflesfixed(){
        shootAllThree();

    }
    private Timer mTimerShoot3close;
    private TimerTask mTtShoot3close;

    protected void shoot3WifflesClose(){
        intakeOff();
        shootWiffleClose();

        mTimerShoot3close = new Timer();
        mTtShoot3close = new TimerTask() {
            public void run() {
                shootAllThree();
            }
        };

        mTimerShoot3close.schedule(mTtShoot3close, 2000);
    }

    /** Condition: true once the AprilTag motif has been seen (stays true after that). */
    @Override
    public boolean motifFound()
    {
        try
        {
            return null != opMode && opMode.detectAprilTag();
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, "motifFound failed: %s", e.toString());
            return false;
        }
    }

//     public void slowToBd(){
//
//     }

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turnA            = 0;        // Desired turning power/speed (-1 to +1)

    public void aprilDriveToBackdrop(){
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection anyDetection : currentDetections) {
            // SDK 12+: detections can be single tags or tag clusters; only single tags have id/metadata.
            if (!(anyDetection instanceof AprilTagSingleDetection)) continue;
            AprilTagSingleDetection detection = (AprilTagSingleDetection) anyDetection;
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id%3 == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                }
            } else {

            }
        }

    }

     public void armDrivePos(){
        robot.arm.moveToLevel(1,1);
     }
     public void armDropSpikePos(){
         robot.arm.moveToCnt(-800,1);

}
    public void outFrontPixel(){

    }

 public void moveToDrive(){
        RobotLog.dd(TAG, "in moveToDrive\n");
     robot.arm.moveToLevel(2, 1);
     robot.slides.setLiftPos(RobotConstants.EL_MIN_ENCODER);
     }



    private Timer mTimerArm;
    private TimerTask mTtArm;

    public void delayedArmToPickup(){
        RobotLog.dd(TAG, "in delayedArmToPickup\n");

        mTimerArm = new Timer();
        mTtArm = new TimerTask() {
            public void run() {
                robot.arm.moveToLevel(0, 1);
                robot.slides.moveToLevel(0);
            }
        };
        mTimerArm.schedule(mTtArm, 1000);
    }

    public void moveArmToZero(){
        RobotLog.dd(TAG, "in moveArmToZero\n");
        robot.arm.moveToCnt(0,1);
    }

    public void moveArmToPickup(){
        RobotLog.dd(TAG, "in moveArmToPickup\n");
        robot.arm.moveToLevel(0, 1);
        robot.slides.moveToLevel(0);
    }

    public void moveSlidesToPickup(){
        RobotLog.dd(TAG, "in moveArmToPickup\n");
        robot.slides.setLiftPos(RobotConstants.EL_MIN_ENCODER);
    }

    public void moveArmTo90(){
        RobotLog.dd(TAG, "in moveArmTo90\n");
        robot.arm.moveToLevel(2, 1);
    }
    public void maxSlides(){
        RobotLog.dd(TAG, "in maxSlides\n");
        robot.slides.setLiftPos(RobotConstants.EL_LEV5);
    }
    public void moveArmToDrop(){
        RobotLog.dd(TAG, "in moveArmToDrop\n");
        robot.arm.moveToLevel(5, 1);
    }
    public void minSlides(){
        RobotLog.dd(TAG,  "in minSlides\n");
        robot.slides.setLiftPos(RobotConstants.EL_MIN_ENCODER);
    }


    @Override
    public void armToDrop(){
        RobotLog.dd(TAG, "in armToDrop\n");
    }
    public void armToDropMid(){

    }

    public void armToDropHigher(){

    }
    public void armToIntake(){

    }
     public void outPixel(){
         if(VERBOSE) { RobotLog.dd(TAG, "in outPixel");}

     }
    public void outPurplePixel(){
        robot.purplePixelDrop = true;

    }

    public void intakes(){

    }
     public void intakesSlow(){

     }

    public void dropPixel(){
        //TODO: add code to make this work
    }

    public void dropPixels(){
        outPixel();
        sleep(500);
//        armToDropHigher();
//        runToDistSensor();
        outPixel();
        sleep(500);
    }
    public void runToDistSensor(){

    }

    public void pickUpPixel(){
        //TODO: add code to make this work
    }

    public void pickUpPixels(){
        pickUpPixel();
        pickUpPixel();
    }

/*
    public void initExtraMovements(ITD_Auton opMode) {
        if ((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.LEFT && startPos == Field.StartPos.START_SAMPLES)  || (alliance == Field.Alliance.RED && teamElement == Route.TeamElement.RIGHT && startPos == Field.StartPos.START_SAMPLES)) {
            //do nothing; the adjustment got baked in
        }else if (startPos == Field.StartPos.START_SPECIMENS&& teamElement == TeamElement.RIGHT && alliance == Field.Alliance.BLUE){
            opMode.doAuton(extraMovementBlueBdRight);
        } else if ((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.RIGHT) || (alliance == Field.Alliance.RED && teamElement == Route.TeamElement.LEFT))
        {
            //do nothing; the adjustment got baked in
        }else if((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.CENTER && startPos == Field.StartPos.START_SAMPLES)  || (alliance == Field.Alliance.RED && teamElement == Route.TeamElement.CENTER))
        {
            //do nothing; the adjustment got baked in
        }
        else if (startPos == Field.StartPos.START_SAMPLES && alliance == Field.Alliance.RED){
            opMode.doAuton(extraMovementRedStacksLeft);
        }
        else if(startPos == Field.StartPos.START_SAMPLES){
            opMode.doAuton(extraMovement2);
        }else if (startPos == Field.StartPos.START_SPECIMENS && teamElement == TeamElement.LEFT && alliance == Field.Alliance.RED){
//            opMode.doAuton(extraMovementRedBdLeft);
        }else if (startPos == Field.StartPos.START_SPECIMENS&& (teamElement == TeamElement.CENTER) && alliance == Field.Alliance.BLUE){
            opMode.doAuton(extraMovementBlueBdCenter);
        }else if (startPos == Field.StartPos.START_SPECIMENS&& teamElement == TeamElement.LEFT && alliance == Field.Alliance.BLUE){
            opMode.doAuton(extraMovementBlueBdLeft);
        }

    }
*/
    public void runRoute(Decode_Auton opMode){

        opMode.doAuton(this);
      //  initExtraMovements(opMode);

//        outPixel();
//        sleep(500);

        //TODO: try to re-add TimeForPixelRun to change what happens based on current time
//        TimeForPixelRun q1 = new TimeForPixelRun(timeForPixelRun, pixelStacks[0]);
//        double time =0;
////        while(q1.isTime(time)) { // if the contents of this loop changes umm TimeForPixelRun needs to also change!

  //      if(PreferenceMgr.getCircuit() >=1) { opMode.doAuton(circuit1); dropPixels(); armToDropHigher(); }
    //    if(PreferenceMgr.getCircuit() >=2) { opMode.doAuton(circuit2); dropPixels();}
      //  if(PreferenceMgr.getCircuit() >=3) { opMode.doAuton(circuit3);}


            // time = find total duration of current route
//            time = calcTimeOrSomething();
//        }

      //  opMode.doAuton(moveToPark);
    }


    @Override
    protected void dashboardConfig(){		//TODO: Fix for ITD
        try {

            HalDashboard dashboard = CommonUtil.getInstance().getDashboard();
            int lnum = 8;
            dashboard.displayText(lnum++, "Start:    " + startPos);
            dashboard.displayText(lnum++, "Park Position:  " + parkPos);
            //dashboard.displayText(lnum++, "Curcuit:    " + curcuit);
            dashboard.displayText(lnum++, "Curcuit 1 " + highways[0] + "," + pixelStacks[0] + "," + highways[1]);
            dashboard.displayText(lnum++, "Curcuit 2 " + highways[2] + "," + pixelStacks[1] + "," + highways[3]);
            dashboard.displayText(lnum++, "Curcuit 3 " + highways[4] + "," + pixelStacks[2] + "," + highways[5]);
        }
        catch (Exception e){}
    }
}
