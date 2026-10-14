package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.util.RobotLog;

import static org.firstinspires.ftc.teamcode.field.Field.Start_Pos.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

/* ==============================================================================================
 * Decode_Route: everything for this season's auton that both the robot and MeepMeep use:
 *   - the poses and tangents (bottom of this file, in initPoses)
 *   - reusable route pieces like shootMotif
 *   - the list of functions a route can call
 *   - picking which route to build (initTrajectories2)
 *
 * Shared by TeamCode and MeepMeepTesting.  It is abstract: each platform makes its own
 * subclass that says what the functions DO:
 *   - TeamCode:        TeamCode's RouteRunner     (runs the shooter, intake, ...)
 *   - MeepMeepTesting: MeepMeep's RouteRunner  (prints what would happen)
 *
 * To add a new function that routes can use (route.addFunction(route::myFunction)):
 *   1. add  protected abstract void myFunction();  in the list below
 *   2. write it in TeamCode's TeamCode's RouteRunner
 *   3. write it in MeepMeepTesting's MeepMeep's RouteRunner (usually just say("myFunction"))
 * The compiler will tell you if you forget step 2 or 3.
 * ============================================================================================== */
public abstract class Decode_Route extends Route
{
  private static final String TAG = "RouteClass";

  public Decode_Route(PositionOption startPos,
                      Field.Park_Pos parkPos,
                      Field.Wiffle_Pos lastLocation,
                      Field.Motif motif,
                      Field.Num_shots numshot
  )
  {
    super(startPos);
    initPoses();
    this.parkPos = parkPos;
    this.lastLocation = lastLocation;
    this.motif = motif;
    Decode_Route.numshot = numshot;
    parkChoice = parkPos;
    buildRoute();
  }

  protected Field.Park_Pos parkPos;
  protected Field.Wiffle_Pos lastLocation;
  protected Field.Motif motif;
  protected static Field.Num_shots numshot;
  protected Field.Route routeStrategy;
  public TeamElement teamElement;
  protected Field.Parks[] highways;
  protected Field.Parks[] pixelStacks;
  protected int numCycles = 0;

  /* ---------------------------------------------------------------------------------------------
   * Functions routes can call with route.addFunction(route::name).
   * RouteRunner (TeamCode) and RouteRunner (MeepMeepTesting) each fill these in.
   * --------------------------------------------------------------------------------------------- */
  protected abstract void shootFar();
  protected abstract void shootWiffleClose();
  protected abstract void shoot1();
  protected abstract void shoot2();
  protected abstract void shoot3();
  protected abstract void shootAllThree();
  protected abstract void shoot3Wiffles();
  public    abstract void shoot3Wifflesfixed();
  protected abstract void wheelOff();
  protected abstract void intakeOn();
  protected abstract void intakeOff();
  protected abstract void intakeonandthreeTransitionsDown();
  protected abstract void transitonUp();
  public    abstract void deliverSample();
  public    abstract void armToDrop();

  /* ---------------------------------------------------------------------------------------------
   * Conditions routes can use with waitUntil / ifBlock / whileBlock, e.g.
   *   route.ifBlock(route::motifFound);
   * --------------------------------------------------------------------------------------------- */
  /** True once the AprilTag motif has been seen. */
  public abstract boolean motifFound();

  /* Placeholder conditions for the conditionals test route (START_FAR_RED6).  For now each one
   * just returns a fixed answer (see both RouteRunners); the real sensor checks come later. */
  /** True once every ball has left the robot. */
  public abstract boolean zeroBalls();
  /** True while the robot is upright. */
  public abstract boolean notTipped();
  /** True when the gate is open. */
  public abstract boolean gateOpen();
  /** True when the row in front of the robot is still full of balls. */
  public abstract boolean rowFull();
  /** True when there's time for another collect lap (whileBlock: must become false eventually). */
  public abstract boolean anotherCycle();

  /** Show the route choices on the driver station (TeamCode's RouteRunner does this; MeepMeep doesn't). */
  protected void dashboardConfig() {}

  protected void goPark()
  {

  }

  /*
  Use the builder functions in Route to build routes in initTrajectories2
  common examples are:
        addLocation(Pose2d, Movement, Heading) - to move the robot to that location
        addFunction(functionName) - to perform a function
        addEvent(WAIT, waitTime) - to add a wait period
        addParkPosition(ParkPos, Pose2d, Movement, Heading) - to define where to park and how to move there
  */

   protected void initTrajectories2()
   {
       RobotLog.dd(TAG, "in InitTrajectories2");
       dashboardConfig();

       Pose2d lastPose;

       RobotLog.dd(TAG, "choosing park");

       if (startPos == START_FAR_RED6)
       {
           // CONDITIONALS TEST ROUTE: uses every conditional (waitUntil, ifBlock, elseIfBlock,
           // elseBlock, endIf, whileBlock, endWhile).  The moves and functions are in the CondTest
           // files; only the control flow is here.  The answers come from the RouteRunners.
           new CondTestShootPreload(this).makeTraj();          // START, shoot from the wall
           waitUntil(this::zeroBalls, 2);

           new CondTestGoCollect(this).makeTraj();             // TANGENT 100, SLOW -> (20, -60, 300)

           ifBlock(this::notTipped);
               addFunction(this::intakeOff);                    // just a function: robot stays put
           elseBlock();
               new CondTestRecoverTipped(this).makeTraj();     // drives somewhere else: route splits
           endIf();

           new CondTestCrossField(this).makeTraj();            // TANGENT 20 -> (60, 12, 0): rejoins

           ifBlock(this::gateOpen, 1.5);                        // waits up to 1.5 s for the gate
               addFunction(this::intakeOn);
               addEvent(Route.Action.WAIT, 1);
               addFunction(this::intakeOff);
           elseIfBlock(this::rowFull);
               addFunction(this::transitonUp);
           elseBlock();
               addFunction(this::wheelOff);
           endIf();

           new CondTestShootNear(this).makeTraj();             // (10, 60, 90), shoot, TANGENT -30 -> (58, -24, 270)

           whileBlock(this::anotherCycle);
               new CondTestCollectCycle(this).makeTraj();      // down the row and back
           endWhile();

           addFunction(this::intakeOff);
           addFunction(this::wheelOff);

           // No park move for the test route: it ends at (58, -24, 270).
           finalizeTrajSeq();
           return;
       }

       if( startPos == START_FAR_RED_SIMPLE)
       {
           SimpleFarRED t1 = new SimpleFarRED(this);
           t1.makeTraj(START_FAR_RED_SIMPLE, parkPos, lastLocation);
       }
        else if (startPos==START_FAR_BLUE_SIMPLE)
       {
           SimpleFarBlue t1 = new SimpleFarBlue(this);
           t1.makeTraj(START_FAR_RED_SIMPLE, parkPos, lastLocation);
       }
       else if (startPos==START_FARFAR_RED_SIMPLE)
       {
           COMPLEXFarRED t1 = new COMPLEXFarRED(this);
           t1.makeTraj(START_FAR_RED_SIMPLE, parkPos, lastLocation);
       }
       else if (startPos==START_FARFAR_BLUE_SIMPLE)
       {
           ComplexFarBlue t1 = new ComplexFarBlue(this);
           t1.makeTraj(START_FAR_RED_SIMPLE, parkPos, lastLocation);
       }




       if( startPos == MAINFARROUTERED)
       {
           MAINFARROUTERED t1 = new MAINFARROUTERED(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }

       else if( startPos == MAINFARROUTEBLUE)
       {
           MAINFARROUTEBLUE t1 = new MAINFARROUTEBLUE(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
       else if (startPos == FARMAINLYNEARRED)
       {
           FARMAINLYNEARRED t1 = new FARMAINLYNEARRED(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
       else if (startPos == FARMAINLYNEARBLUE)
       {
           FARMAINLYNEARBLUE t1 = new FARMAINLYNEARBLUE(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
       else if( startPos == START_WALL_RED4)
       {
           Route4RED t1 = new  Route4RED(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
       else if( startPos == START_WALL_BLUE4)
       {
           Route4BLUE t1 = new Route4BLUE(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
       else if( startPos == START_HUMANONLY)
       {
           RouteHumanOnly t1 = new RouteHumanOnly(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
       else if( startPos == START_FAR_FAR_HUMAN_BLUE)
       {
           FarStayFarBlueHuman t1 = new FarStayFarBlueHuman(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
       else if( startPos == START_GOAL_ROUTE_RED)
       {
           GOALROUTERED t1 = new GOALROUTERED(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
       else if( startPos == START_GOAL_ROUTE_BLUE)
       {
           GOALROUTEBLUE t1 = new GOALROUTEBLUE(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
       else if( startPos == START_FAR_RED6)    // not reached: START_FAR_RED6 runs the conditionals test route above
       {
           ROUTE6REDT t1 = new  ROUTE6REDT(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }else if( startPos == START_FAR_BLUE6)
       {
           ROUTE6BLUET t1 = new ROUTE6BLUET(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
       else if (startPos == STARTFARSTAYFARBLUE){
           FarStayFarBlue t1 = new FarStayFarBlue(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }
         else if (startPos == FARSTAYFARRED){
             FarStayFarRed t1 = new FarStayFarRed(this);
           t1.makeTraj(startPos, parkPos, lastLocation);
       }

       MoveToParkPOSDECODE t1 = new MoveToParkPOSDECODE(this);
       t1.makeTraj(parkPos,alliance);

       lastPose = this.getEnd();
       RobotLog.dd(TAG, "Park choice %s",parkPos);

       RobotLog.dd(TAG, "route made");

/*
       RobotLog.dd(TAG, "making park");
       try{
           Pose2d parkStrt;
            if(PreferenceMgr.getCircuit() == 0){
                parkStrt = new Pose2d(dropOnTapeThenBackdrop.getLastPose().getX(), dropOnTapeThenBackdrop.getLastPose().getY(), dropOnTapeThenBackdrop.getLastPose().getHeading());
            }else{
                parkStrt = new Pose2d(circuit1.getLastPose().getX(), circuit1.getLastPose().getY(), circuit1.getLastPose().getHeading());
            }

           moveToPark.addLocation(parkStrt, START, HEAD_CONSTANT);
       }catch(Exception e){ RobotLog.ww(TAG,"unable to add lastPose(2)");}
       MoveToPark t5 = new MoveToPark(moveToPark);
       t5.makeTraj(parkPos, alliance);
       RobotLog.dd(TAG, "park made");

       createExtraMovementRoutes();
*/
        //Always do this at the end of initTrajectories2
       // finalizeAllTrajSeqs();
	   finalizeTrajSeq();
    }

    protected void shootMotif(int greenPos, Pose2d pos1){
//        double theta = pos1.getHeading() - Math.toDegrees(90);
//        Pose2d pos2 = new Pose2d(pos1.getX()+Math.cos(theta)*DRIVE_TO_SHOOT_AMT, pos1.getY()+Math.sin(theta)*DRIVE_TO_SHOOT_AMT, pos1.getHeading());
//        Pose2d pos3 = new Pose2d(pos2.getX()+Math.cos(theta)*DRIVE_TO_SHOOT_AMT, pos2.getY()+Math.sin(theta)*DRIVE_TO_SHOOT_AMT, pos2.getHeading());
       addEvent(Route.Action.WAIT, .75);
        switch (motif)
       {
           case SHOOTGPP:

               switch (greenPos){
                   case 1:
                            addFunction(this::shoot1);
                            break;
                   case 2:
                            addFunction(this::shoot2);
                            break;
                   case 3:
                            addFunction(this::shoot3);
                            break;
               }
               addEvent(Route.Action.WAIT, .5);
               switch (greenPos){
                   case 1:
                       addFunction(this::shoot2);

                       addFunction(this::shoot3);
                       break;
                   case 2:
                       addFunction(this::shoot1);

                       addFunction(this::shoot3);
                       break;
                   case 3:
                       addFunction(this::shoot1);
                       addFunction(this::shoot2);
                       break;
               }
               addEvent(Route.Action.WAIT, 0.34);
               break;
           case SHOOTPGP:
               switch (greenPos){
                   case 1:
                       addFunction(this::shoot2);
                       break;
                   case 2:
                   case 3:
                       addFunction(this::shoot1);
                       break;
               }
               addEvent(Route.Action.WAIT, .5);
               switch (greenPos){
                   case 1:
                       addFunction(this::shoot1);
                       break;
                   case 2:
                       addFunction(this::shoot2);
                       break;
                   case 3:
                       addFunction(this::shoot3);
                       break;
               }
               addEvent(Route.Action.WAIT, .50);
               switch (greenPos){
                   case 1:
                   case 2:
                       addFunction(this::shoot3);
                       break;
                   case 3:
                       addFunction(this::shoot2);
                       break;
               }
               addEvent(Route.Action.WAIT, 0.34);

               break;
           case SHOOTPPG:
               switch (greenPos){
                   case 1:
                       addFunction(this::shoot2);
                   addEvent(Route.Action.WAIT, .75);
                       addFunction(this::shoot3);
                       break;
                   case 2:
                       addFunction(this::shoot1);
                       addEvent(Route.Action.WAIT, .75);
                       addFunction(this::shoot3);
                       break;
                   case 3:

                       addFunction(this::shoot1);

                       addEvent(Route.Action.WAIT, .75);

                       addFunction(this::shoot2);
                       break;
               }
               addEvent(Route.Action.WAIT, .75);
               switch (greenPos){
                   case 1:
                       addFunction(this::shoot1);
                       break;
                   case 2:
                       addFunction(this::shoot2);
                       break;

                   case 3:
                       addFunction(this::shoot3);
                       break;
               }
               addEvent(Route.Action.WAIT, .5);
               break;
       }
     }

  /* =============================================================================================
   * POSES: every Pose2d and tangent the routes use (route.startSmallTri, route.ninety, ...).
   * To add a pose: declare it with the others below, then set it in initPoses().
   * ============================================================================================= */
     private void initPoses()
     {
           botLen = RobotConstants.BOT_LEN;
           botWid = RobotConstants.BOT_WID;

           double rightSideLineUpToBorderAdjustment = 0.5;


         botBackToCtr = botLen / 2.0;
         botSideToCtr = botWid / 2.0;

         hubRad = 9.0;
         double backOffset = 0.0;




             strtY = -1.5 * Field.tileWidth;


         strtX =  3.0f * Field.tileWidth - botBackToCtr;

         strtH = Math.toRadians(180.0);

         start = new Pose2d(sx * strtX, sy * strtY, flip + sh* strtH);
         ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //Put new poses here
        startSample = new Pose2d(sx * 5, 63, flip + sh*Math.toRadians(270));
        //specimen points
        blueFar  = new Pose2d(sx * -11, -60, flip + sh*Math.toRadians(270));
        shootPreWhiffles  = new Pose2d(sx * -14, -57, flip + sh*Math.toRadians(115));
        redWall  = new Pose2d(sx * 36, 62, flip + sh*Math.toRadians(270));
        aroundPartner = new Pose2d(sx * 13, 43.1, flip + sh*Math.toRadians(30));
        shootZonefornear = new Pose2d(sx * 30, 29, flip + sh*Math.toRadians(53));
        shootZonefornearblue = new Pose2d(sx * -30, 20, flip + sh*Math.toRadians(-53));

        intakeGoal = new Pose2d(sx * 31.3, 8, flip + sh*Math.toRadians(90));
        intakeGoalWhiffles = new Pose2d(sx * 55 , 8, flip + sh*Math.toRadians(90));
        intakeGate = new Pose2d(sx * 31.3, -14, flip + sh*Math.toRadians(90));
        intakeGateWhiffles = new Pose2d(sx * 52, -14, flip + sh*Math.toRadians(90));
        shootGateWhiffles = new Pose2d(sx * 24, 43.1, flip + sh*Math.toRadians(30));
        intakePark = new Pose2d(sx * 31.3, -37, flip + sh*Math.toRadians(90));
        intakeParkWhiffles = new Pose2d(sx * 52, -37, flip + sh*Math.toRadians(90));
        shootParkWhiffles = new Pose2d(sx * 13, 43.1, flip + sh*Math.toRadians(30));
        redGoal = new Pose2d(sx * 48.41, 47.94, flip + sh*Math.toRadians(38.5));
        blueGoal = new Pose2d(sx * -45.41, 53, flip + sh*Math.toRadians(128.5));
        blueWall  = new Pose2d(sx * -36, 62, flip + sh*Math.toRadians(270));
        aroundPartnerBlue = new Pose2d(sx * -13, 43.1, flip + sh*Math.toRadians(150));


        intakeGateBlue = new Pose2d(sx * -31.3, -14, flip + sh*Math.toRadians(-90));
        intakeGateWhifflesBlue = new Pose2d(sx * -52, -14, flip + sh*Math.toRadians(-90));
        shootGateWhifflesBlue = new Pose2d(sx * -12, 13,flip + sh*Math.toRadians(130.37));
        shootGateWhifflesBluebackup = new Pose2d(sx * -12, 0,flip + sh*Math.toRadians(130.37));
        aroundWhiffles = new Pose2d(sx * -28, -5, flip + sh*Math.toRadians(150));

        intakeGoalBlue = new Pose2d(sx * -20, 25, flip + sh*Math.toRadians(255));



        shootGoalWhifflesBlue = new Pose2d(sx * -20.03, 10.72, flip + sh*Math.toRadians(130.37));
        intakeGoalWhifflesBlue = new Pose2d(sx * -59 , 28, flip + sh*Math.toRadians(255));



        intakeParkBlue = new Pose2d(sx * -13, -29.4, flip + sh*Math.toRadians(270));
        intakeParkWhifflesBlue = new Pose2d(sx * -51, -29.4, flip + sh*Math.toRadians(270));
        shootParkWhifflesBlue = new Pose2d(sx * -13, 43.1, flip + sh*Math.toRadians(150));
        //ITD poses
        startSmallTri = new Pose2d(sx * 10, -65, flip + sh*Math.toRadians(90));
        simpleShootFar = new Pose2d(sx * 17, -56, flip + sh*Math.toRadians(70));

        startSmallTriBlue = new Pose2d(sx * -10, -60, flip + sh*Math.toRadians(90));
        humanZone = new Pose2d(sx * -55, -50, flip + sh*Math.toRadians(270));
        shootFarPosBLUE = new Pose2d(sx * -20, -53, flip + sh*Math.toRadians(109.2));
        shootFarfarPosBLUE = new Pose2d(sx * -20, -52, flip + sh*Math.toRadians(109));

        shootFarPos = new Pose2d (sx * 15, -58, flip + sh*Math.toRadians(70));
        shootFarPosB = new Pose2d (sx * -16, -58, flip + sh*Math.toRadians(290));
        shootNearPos = new Pose2d (sx * 10, 1, flip + sh*Math.toRadians(70));
        shootNearPosB = new Pose2d (sx * 10, 1, flip + sh*Math.toRadians(290));
        PrepPosIntake1 = new Pose2d(sx * 58, -50 , flip + sh*Math.toRadians(45));
        PrepPosIntake1BLUE = new Pose2d(sx * -58, -50 , flip + sh*Math.toRadians(-75));

        intookpos1 = new Pose2d(sx * 60.3, -59 , flip + sh*Math.toRadians(0));
        intookpos1BLUE = new Pose2d(sx * -59, -60 , flip + sh*Math.toRadians(-45));
        intookpos1BLUEDONE = new Pose2d(sx * -59, -60 , flip + sh*Math.toRadians(-40));

        intakepos2 = new Pose2d(sx * 25, -40 , flip + sh*Math.toRadians(90));
        intakepos2B = new Pose2d(sx * -29, -28 , flip + sh*Math.toRadians(270));
        intakeloading2 = new Pose2d(sx * 50, -40 , flip + sh*Math.toRadians(90));
        intakeloading2B = new Pose2d(sx * -49, -28 , flip + sh*Math.toRadians(270));
        intakepos3 = new Pose2d(sx * 25, -17 , flip + sh*Math.toRadians(90));
        intakepos3B = new Pose2d(sx * -29, -5 , flip + sh*Math.toRadians(270));
        intakeloading3 = new Pose2d(sx * 50, -17 , flip + sh*Math.toRadians(90));
        intakeloading3B = new Pose2d(sx * -49, -5 , flip + sh*Math.toRadians(270));
        shootPos3 = new Pose2d(sx * -5, 18 , flip + sh*Math.toRadians(90));
        intakepos4 = new Pose2d(sx * 25, 7 , flip + sh*Math.toRadians(90));
        intakeloading4 = new Pose2d(sx * 50, 7 , flip + sh*Math.toRadians(90));
        preleverwiffle= new Pose2d(sx * 15, -17 , flip + sh*Math.toRadians(90));
        intakewifflelever = new Pose2d(sx * 52, -16 , flip + sh*Math.toRadians(90));
        hitlever = new Pose2d(sx * 56, -6.65 , flip + sh*Math.toRadians(90));
        prehitlever = new Pose2d(sx * 42, -6.65 , flip + sh*Math.toRadians(90));

        prenearpos = new Pose2d (sx * 17,-10,flip +sh*Math.toRadians(40));
        nearpos = new Pose2d (sx * 15,37,flip +sh*Math.toRadians(35));
        pregotoparkwiffle = new Pose2d (sx * 15,-40,flip +sh*Math.toRadians(90));
        gotoparkwiffle = new Pose2d (sx * 50,-40,flip +sh*Math.toRadians(90));
        pregotogoalwiffles= new Pose2d (sx * 31,7,flip +sh*Math.toRadians(90));
        gotogoalwiffles= new Pose2d (sx * 54.5,7,flip +sh*Math.toRadians(90));

        parkInside1Red = new Pose2d(sx * 17,37,flip +sh*Math.toRadians(90));
        parkInside1Blue = new Pose2d(sx * -16,38,flip +sh*Math.toRadians(90));

        parkInside2Red = new Pose2d(sx * 15,-18,flip +sh*Math.toRadians(90));
        parkInside2Blue = new Pose2d(sx * -15,-18,flip +sh*Math.toRadians(90));

        parkOutside1Red =new Pose2d(sx * 30,58,flip +sh*Math.toRadians(90));
        parkOutside1Blue =new Pose2d(sx * -30,58,flip +sh*Math.toRadians(90));

        parkOutside2Red =new Pose2d(sx * 52,30,flip +sh*Math.toRadians(90));
        parkOutside2BLUE =new Pose2d(sx * -52,30,flip +sh*Math.toRadians(90));
        tempParkRed = new Pose2d(sx * 57,-50,flip +sh*Math.toRadians(90));
        tempParkBlue = new Pose2d(sx * -30,-40,flip +sh*Math.toRadians(50));

        startRedFar = new Pose2d(sx * 20, -60, flip + sh*Math.toRadians(300));     // conditionals test route (was 20,-60,90)

        collect2 = new Pose2d (sx * 56, -46, flip + sh*Math.toRadians(67));
        helpcollect4 = new Pose2d (sx * 68, -48, flip + sh*Math.toRadians(65));
        helpcollect3 = new Pose2d (sx * 62, -47, flip + sh*Math.toRadians(66));
        helpcollect2= new Pose2d (sx * 50, -45, flip + sh*Math.toRadians(68));


        collect3 = new Pose2d (sx * 56, -20, flip + sh*Math.toRadians(67));
        helpCollectLever2= new Pose2d (sx * 50, -19, flip + sh*Math.toRadians(68));
        helpCollectLever3 = new Pose2d (sx * 62, -21, flip + sh*Math.toRadians(66));
        helpCollectLever4 = new Pose2d (sx * 68, -22, flip + sh*Math.toRadians(65));




        moveToParkBlue = new Pose2d (sx * -20, -17, flip + sh*Math.toRadians(270));
        collect2Blue = new Pose2d (sx * -56, -41, flip + sh*Math.toRadians(267));
        helpcollect5Blue = new Pose2d (sx * -62, -17, flip + sh*Math.toRadians(265));
        helpcollect4Blue = new Pose2d (sx * -68, -41, flip + sh*Math.toRadians(265));
        helpcollect3Blue = new Pose2d (sx * -62, -41, flip + sh*Math.toRadians(266));
        helpcollect2Blue= new Pose2d (sx * -50, -41, flip + sh*Math.toRadians(268));




        nearLeaver = new Pose2d (sx * 17, -15, flip + sh*Math.toRadians(90));
        shootNear = new Pose2d (sx * 15, 37, flip + sh*Math.toRadians(25));
        shootNearForSpline = new Pose2d (sx * 30, 37, flip + sh*Math.toRadians(30));
        moveToHumanPlayerZone = new Pose2d (sx * 61, -40, flip + sh*Math.toRadians(360));
        collect1 = new Pose2d (sx * 61, -60, flip + sh*Math.toRadians(360));
        shootNear2 = new Pose2d (sx * 30, 37, flip + sh*Math.toRadians(30));
        startSpecimenSide = new Pose2d(sx * -5, 63.25, flip + sh*Math.toRadians(270));
		startSpecimen = new Pose2d(sx * -5, 63, flip + sh*Math.toRadians(270));
        specimenPark1 = new Pose2d(sx * -61, 62, flip + sh*Math.toRadians(270));
        specimen3 = new Pose2d(sx * -59.01, 55, flip + sh*Math.toRadians(245));
        specimen2 = new Pose2d(sx * -59, 55, flip + sh*Math.toRadians(270));
        dropPreSpecimen = new Pose2d(sx * -5, 35, flip + sh*Math.toRadians(270));
        forward = new Pose2d(sx * -5, 35, flip + sh*Math.toRadians(270));
        startCSRedHigh = new Pose2d(sx * 63.25, 16, flip + sh*Math.toRadians(180));
        specimen1PosPosToPush = new Pose2d(sx * -27.7, 35, flip + sh*Math.toRadians(270));
        specimen1PosToPush = new Pose2d(sx * -48, 10, flip + sh*Math.toRadians(270));
       specimen1 = new Pose2d(sx * -48,55, flip + sh*Math.toRadians(270));
       corner = new Pose2d(sx * -55,55, flip + sh*Math.toRadians(310));
        corner2 = new Pose2d(sx * -55,55, flip + sh*Math.toRadians(310));
        corner3 = new Pose2d(sx * -55,55, flip + sh*Math.toRadians(310));
        startCSRedLow = new Pose2d(sx*63.25, -32, flip + sh*Math.toRadians(180));
        startCBlueHigh = new Pose2d(sx * -63.25, 16, flip + sh*Math.toRadians(0));
        startCSBlueLow = new Pose2d(sx * -63.25, -15.5, flip + sh*Math.toRadians(0));
         purplePixelPlaceCenterTop = new Pose2d(sx * 28, 23.5, flip + sh*Math.toRadians(230));
         dropCenterPixel = new Pose2d(sx * 10, 60, flip + sh*Math.toRadians(90));  // conditionals test route (was 6,65,270)
        hangSpecimen = new Pose2d(sx * 5, 44.9, flip + sh*Math.toRadians(270));
        moveBackFromSpecimen = new Pose2d(sx * 9, 55, flip + sh*Math.toRadians(270));
        sample1 = new Pose2d(sx * 47, 53.5, flip + sh*Math.toRadians(270));
        deliverSampleToBasket = new Pose2d(sx * 55, 61.25 , sh*Math.toRadians(50));
        deliverSample2ToBasket = new Pose2d(sx * 57.5, 64.2, sh*Math.toRadians(45));
        deliverSample3ToBasket = new Pose2d(sx * 56.5, 66.5, sh*Math.toRadians(53));
        sample2 = new Pose2d(sx * 54, 54.5, flip + sh*Math.toRadians(-100));// x is 58 y is 53.5 heading is -119.9
        rotateToSample2 = new Pose2d(sx * 55, 56, flip + sh*Math.toRadians(-90));
        turnStraight = new Pose2d (sx * 56, 54, flip + sh*Math.toRadians(-90));
        sample3 = new Pose2d(sx * 41.6, 27, flip + sh*Math.toRadians(-4));
        park = new Pose2d(sx * 28, 10, flip + sh*Math.toRadians(-180));
        positionToPark = new Pose2d(sx * 40,10 , flip + sh*Math.toRadians(-90));
        waitForHumanPlayer = new Pose2d(sx * -48, 48, flip + sh*Math.toRadians(89.9));
		waitForHumanPlayer2 = new Pose2d(sx * -46, 45.75, flip + sh*Math.toRadians(89.9));
        waitForHumanPlayer3 = new Pose2d(sx * -47.25, 43.5, flip + sh*Math.toRadians(89.9));
        park2 = new Pose2d(sx * -70, 63, flip + sh*Math.toRadians(-90));
        subPos = new Pose2d(sx * -5, 35, flip + sh*Math.toRadians(270));
        startRedWall = new Pose2d(sx * 30, 62, flip + sh*Math.toRadians(270));
        startBlueWall = new Pose2d(sx * -30, 62, flip + sh*Math.toRadians(270));
        shootNearBlue = new Pose2d(sx * -25, 50, flip + sh*Math.toRadians(150));
        goNear2Blue = new Pose2d(sx * -25, -30, flip + sh*Math.toRadians(270));
        goNear3Blue = new Pose2d(sx * -25, -5, flip + sh*Math.toRadians(270));
        collect3Blue = new Pose2d(sx * -55, -5, flip + sh*Math.toRadians(270));
        goNear1Blue = new Pose2d(sx * -61, -43, flip + sh*Math.toRadians(180));
        collect1Blue = new Pose2d(sx * -61, -60, flip + sh*Math.toRadians(180));
        shootNearBlueSpline = new Pose2d(sx * -30, 40, flip + sh*Math.toRadians(150));
        startBlueFar= new Pose2d(sx * -20, -55, flip + sh*Math.toRadians(90));
        dropCenterPixelAdj = new Pose2d(sx * 27, 57.75, flip + sh*Math.toRadians(270));
        dropCenterPixelStacksAdj = new Pose2d(sx * 20, 50, flip + sh*Math.toRadians(270));
        dropCenterPixelLeftAdj = new Pose2d(sx *31.5, 50.35, flip + sh*Math.toRadians(270));
        dropCenterPixelCenterAdjBlue = new Pose2d(sx *35, 46, flip + sh*Math.toRadians(270));
        dropCenterPixelLeftAdjBlue = new Pose2d(sx *35.5, 43, flip + sh*Math.toRadians(270));
        dropCenterPixelLeftAdjRed= new Pose2d(sx *30, 51.35, flip + sh*Math.toRadians(270));
        dropCenterPixelRightAdjBlue = new Pose2d(sx *30, 54, flip + sh*Math.toRadians(270));
         dropCenterPixelBackwards = new Pose2d(sx * 32.5, 48, flip + sh*Math.toRadians(270));
        dropLeftPixelBackwardsBlue = new Pose2d(sx * 34.25, 63.5, flip + sh*Math.toRadians(270));
        dropRightPixelBackwardsRed = new Pose2d(sx * 40.5, 62.35, flip + sh*Math.toRadians(270));
        dropRightPixelBackwardsBlue = new Pose2d(sx * 25, 56.5, flip + sh*Math.toRadians(270));
        dropLeftPixelBackwardsRed = new Pose2d(sx * 27, 61, flip + sh*Math.toRadians(270));
        dropCenterPixelBackwardsBlue = new Pose2d(sx * 31.75, 56, flip + sh*Math.toRadians(270));
        dropCenterPixelBackwardsRed = new Pose2d(sx * 33.5, 61.5, flip + sh*Math.toRadians(270));
        dropCenterPixelBackwardsRedBd = new Pose2d(sx * 36, 59.5, flip + sh*Math.toRadians(270));
        dropRightPixelBackwardsRedBd = new Pose2d(sx * 42.5, 57.5, flip + sh*Math.toRadians(270));
        dropLeftPixelBackwardsRedBd = new Pose2d(sx * 17.5, 56.5, flip + sh*Math.toRadians(270));

        dropCenterPixelBackwardsLeftBackdrop = new Pose2d(sx * 34.5, 40, flip + sh*Math.toRadians(270));
        whatchamacallit = new Pose2d(sx * 36, -36, flip + sh*Math.toRadians(270));
        pickUpPixelStackLeft = new Pose2d(sx * 35, -62, flip + sh*Math.toRadians(-90));
        pickUpPixelStackLeftBlue = new Pose2d(sx * 35, -62, flip + sh*Math.toRadians(-90));
        pickUpPixelStackLeftLeft = new Pose2d(sx * 36.5, -63.5, flip + sh*Math.toRadians(-90));
        pickUpPixelStackLeftCenter = new Pose2d(sx * 40, -58, flip + sh*Math.toRadians(-90));
        dropLeftPixel = new Pose2d(sx * 33, 53.8, flip + sh* Math.toRadians(270));
        dropRightPixelBlue = new Pose2d(sx * 27, 52.5, flip + sh* Math.toRadians(270));
        dropRightPixel = new Pose2d(sx * 47, 54, flip + sh*Math.toRadians(270));
        dropRightPixelCircuit = new Pose2d(sx * 45, 54.5, flip + sh*Math.toRadians(270));
        dropLeftPixelBlue = new Pose2d(sx * 45, 53, flip + sh*Math.toRadians(270));
        pickUpPixelStackRight = new Pose2d(sx * 34.5, -59, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightCircuit = new Pose2d(sx * 36, -58.5, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightRedLeft = new Pose2d(sx * 31, -57, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightCircuitBlue = new Pose2d(sx * 30, -59, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightCircuitBlueCenter = new Pose2d(sx * 30.5, -59, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightCircuitBlueRight = new Pose2d(sx * 33.5, -55, flip + sh*Math.toRadians(-90));
        pickUpPixelStackRightLeft = new Pose2d(sx * 35, -61, flip + sh*Math.toRadians(-90));
        pickUpPixelStackCenter = new Pose2d(sx * 10, -62, flip + sh*Math.toRadians(270));   // conditionals test route start (was 12,-59,-90)
        pickUpPixelStackCenterBlue = new Pose2d(sx * 17, -58, flip + sh*Math.toRadians(-90));
        backAwayBackDrop = new Pose2d(sx * 35, 43, flip + sh*Math.toRadians(180));
        underRigging = new Pose2d(sx * 66, -43, flip + sh*Math.toRadians(270));
        pointForRobotToPixelStack = new Pose2d(sx * 58, -44, flip + sh*Math.toRadians(270));      // conditionals test route (was 59,-42,270)
        pointForRobotToPixelStackBlue = new Pose2d(sx * 58, -24, flip + sh*Math.toRadians(270));  // conditionals test route (was 62,-24,270)
        aboveRigging = new Pose2d(sx * 61, 3, flip + sh*Math.toRadians(270));
        aboveRiggingBackwards = new Pose2d(sx * 61, 8, flip + sh*Math.toRadians(90));
        underCenter = new Pose2d(sx * 36, -9, flip + sh*Math.toRadians(270));
        pointcuzwhynot = new Pose2d(sx * 36, -37, flip + sh*Math.toRadians(270));
        underdoor = new Pose2d(sx*11, -63, flip + sh*Math.toRadians(270));
        underMiddlePoint = new Pose2d(sx*35, -22, flip + sh*Math.toRadians(270));
        bottom =new Pose2d( sx*30,24, flip + sh*Math.toRadians(0));
        middle=new Pose2d( sx* 25,18, flip + sh*Math.toRadians(180));
        top=new Pose2d( sx*35,21, flip + sh*Math.toRadians(90));
        parkDoor = new Pose2d(sx*10, 44.5, flip + sh*Math.toRadians(270));
        parkWall = new Pose2d(sx*68, 44.5, flip + sh*Math.toRadians(270));
        teamElementLeft = new Pose2d(sx*28, 8, flip + sh*Math.toRadians(90));
        teamElementRight = new Pose2d(sx*36, 23, flip + sh*Math.toRadians(20));
        teamElementCenter = new Pose2d(sx* 27.5, 15, flip + sh*Math.toRadians(0));
        teamElementLeftLow= new Pose2d(sx*32, -35, flip + sh*Math.toRadians(50));
        teamElementRightLow = new Pose2d(sx*30, -21, flip + sh*Math.toRadians(270));
        teamElementCenterLow = new Pose2d(sx*28, -29, flip + sh*Math.toRadians(0));
        extraPointLow = new Pose2d(sx*32, -30, flip + sh*Math.toRadians(270));
        extraPointHigh = new Pose2d(sx*36, 23, flip + sh*Math.toRadians(90));


        starting_point_top = new Pose2d(sx*60,12,flip + sh*Math.toRadians(0));   // conditionals test route (was 58,18,180)
        staring_point = new Pose2d(sx*50,-31,flip + sh*Math.toRadians(180));
        team_element = new Pose2d(sx*30,-28,flip + sh*Math.toRadians(90));
        white_pixel_middle = new Pose2d(sx*22,-62,flip + sh*Math.toRadians(270));
        center = new Pose2d(sx*6,3,flip + sh*Math.toRadians(270));
        back_drop = new Pose2d(sx*30, 56,flip + sh*Math.toRadians(270));
        red_side =new Pose2d(sx*58,3,flip + sh*Math.toRadians(270));

        shootfaronredpre = new Pose2d ( 15, -60, Math.toRadians(70));
        shootfaronred = new Pose2d ( 15, -58, Math.toRadians(70));
        shootfarfaronred = new Pose2d (16, -57,Math.toRadians(70));
        shootfarfarfaronred = new Pose2d (15, -57,Math.toRadians(70));
        shootfarfarfarfaronred = new Pose2d (15, -58,Math.toRadians(66.5));
        moveToPark = new Pose2d( 29,-36,Math.toRadians(90));
        helpcollect5 = new Pose2d(57,-36,Math.toRadians(90));


        moveToLeverblue = new Pose2d(-17.25, -2 ,Math.toRadians(251.65));
        helpCollectLever5blue = new Pose2d(-56, 10,Math.toRadians(255));
        helpCollectLever5bluefar = new Pose2d(-69, 8.3,Math.toRadians(255));




        moveToLever = new Pose2d(31, -20 ,Math.toRadians(104.35));
        helpCollectLever5 = new Pose2d(61.36, -14,Math.toRadians(104.49));
        shootGoalWhiffles = new Pose2d(12,8,Math.toRadians(49.63));
        shootGoalWhifflesinside = new Pose2d(10,9,Math.toRadians(49.63));
        route5movementfromstart= new Pose2d(25,37,Math.toRadians(35));



        /* Qualifier Route Points */
        if(RobotConstants.bot == RobotConstants.Chassis.B7252) {
            moveAwayFromWallRedBackdrop = new Pose2d(50, 15, Math.toRadians(180));
            moveAwayFromWallBlueBackdrop = new Pose2d(-50, 15, Math.toRadians(0));

            moveAwayFromWallRedStacks = new Pose2d(52, -35, Math.toRadians(180));
            moveAwayFromWallBlueStacks = new Pose2d(-54, -35, Math.toRadians(0));

            moveTowardsRedBackdrop = new Pose2d(64.5, 20, Math.toRadians(270));
            moveTowardsBlueBackdrop = new Pose2d(-60, 8, Math.toRadians(270));

            moveTowardsRedBackdropLft = new Pose2d(59, 15, Math.toRadians(270));

            moveTowardsBlueBackdropHdAdj = new Pose2d(-35, 38, Math.toRadians(245));
            moveTowardsRedBackdropHdAdj = new Pose2d(40, 38, Math.toRadians(295));

            moveTowardsRedBackdropHdAdjLft = new Pose2d(35, 30, Math.toRadians(295));
            moveTowardsRedBackdropHdAdjRt = new Pose2d(30, 30, Math.toRadians(295));

            dropPixelRedRightTapeBackdrop = new Pose2d(34, 24, Math.toRadians(0));
            dropPixelRedCenterTapeBackdrop = new Pose2d(32, 15, Math.toRadians(0));
            dropPixelRedLeftTapeBackdrop = new Pose2d(32, 5, Math.toRadians(90));

            dropPixelRedLeftTapeBackdropAdj = new Pose2d(32, 9, Math.toRadians(90));
            dropPixelRedRightTapeBackdropAdj = new Pose2d(36, 27, Math.toRadians(0));

            dropPixelBlueRightTapeBackdrop = new Pose2d(-33, 5, Math.toRadians(90));
            dropPixelBlueCenterTapeBackdrop = new Pose2d(-29, 15, Math.toRadians(180));
            dropPixelBlueLeftTapeBackdrop = new Pose2d(-33, 18, Math.toRadians(270));

            dropPixelBlueCenterBackTapeAdj = new Pose2d(-33, 15, Math.toRadians(-180));
            dropPixelRedCenterBackTapeAdj = new Pose2d(32, 18, Math.toRadians(0));

            dropPixelRedRightTapeStacks = new Pose2d(32, -30.5, Math.toRadians(270));
            dropPixelRedCenterTapeStacks = new Pose2d(21.5, -45, Math.toRadians(270));
            dropPixelRedLeftTapeStacks = new Pose2d(32, -46, Math.toRadians(0));

            dropPixelRedRightTapeStacksAdj = new Pose2d(32, -46, Math.toRadians(270));

            dropPixelRedCenterStkTapeAdj = new Pose2d(26, -45, Math.toRadians(270));
            dropPixelBlueCenterStkTapeAdj = new Pose2d(-27, -46, Math.toRadians(270));

            dropPixelBlueRightTapeStacks = new Pose2d(-32, -52, Math.toRadians(270));
            dropPixelBlueCenterTapeStacks = new Pose2d(-20, -42, Math.toRadians(270));
            dropPixelBlueLeftTapeStacks = new Pose2d(-33, -30, Math.toRadians(270));

            moveFromBlueRightTapeStacks = new Pose2d(-32, -57.5, Math.toRadians(270));
            moveFromBlueLeftTapeStacks = new Pose2d(-32, -44, Math.toRadians(270));
            moveFromBlueCenterTapeStacks = new Pose2d(-24, -52, Math.toRadians(270));

            moveFromRedLeftTapeStacks = new Pose2d(59, -50, Math.toRadians(270));
            moveFromRedRightTapeStacks = new Pose2d(32, -40, Math.toRadians(270));
            moveFromRedCenterTapeStacks = new Pose2d(24, -52, Math.toRadians(230));

            dropOnBackdropBlueLeftBackdrop = new Pose2d(-46.7, 52.5, Math.toRadians(270));
            dropOnBackdropBlueRightBackdrop = new Pose2d(-25, 53.5, Math.toRadians(270));
            dropOnBackdropBlueCenterBackdrop = new Pose2d(-37, 53.5, Math.toRadians(270));

            dropOnBackdropBlueLeftBackdropHi = new Pose2d(-43, 55.5, Math.toRadians(270));
            dropOnBackdropBlueRightBackdropHi = new Pose2d(-25, 55.5, Math.toRadians(270));
            dropOnBackdropBlueCenterBackdropHi = new Pose2d(-37, 55.5, Math.toRadians(270));

            dropOnBackdropRedLeftBackdrop = new Pose2d(28, 52, Math.toRadians(270));
            dropOnBackdropRedRightBackdrop = new Pose2d(45.5, 56.3, Math.toRadians(270));
            dropOnBackdropRedCenterBackdrop = new Pose2d(36.5, 55.5, Math.toRadians(270));

            moveAwayFromCRedBackdropTape = new Pose2d(38, 15, Math.toRadians(0));
            moveAwayFromLRedBackdropTape = new Pose2d(32, 15, Math.toRadians(90));
            moveAwayFromRRedBackdropTape = new Pose2d(41, 24, Math.toRadians(0));

            moveAwayFromCBlueBackdropTape = new Pose2d(-45, 15, Math.toRadians(180));
            moveAwayFromLBlueBackdropTapeAdj = new Pose2d(-33, 14, Math.toRadians(270));
            moveAwayFromRBlueBackdropTape = new Pose2d(-32, 15, Math.toRadians(90));

            moveAwayFromLBlueBackdropTape = new Pose2d(-50, 14, Math.toRadians(270));

            reverseFromRedBackdropBk = new Pose2d(36, 35, Math.toRadians(270));
            reverseFromBlueBackdropBk = new Pose2d(-36, 42, Math.toRadians(270));

            reverseFromRedBackdropStk = new Pose2d(36, 42, Math.toRadians(310));
            reverseFromBlueBackdropStk = new Pose2d(-36, 42, Math.toRadians(230));

            dropOnBackdropBlueLeftStacks = new Pose2d(-39, 55.5, Math.toRadians(270));
            dropOnBackdropBlueRightStacks = new Pose2d(-16.5, 59, Math.toRadians(270));
            dropOnBackdropBlueCenterStacks = new Pose2d(-31.5, 57, Math.toRadians(270));

            dropOnBackdropBlueLeftStacksHi = new Pose2d(-39, 57, Math.toRadians(270));
            dropOnBackdropBlueRightStacksHi = new Pose2d(-16.5, 60, Math.toRadians(270));
            dropOnBackdropBlueCenterStacksHi = new Pose2d(-31.5, 58.5, Math.toRadians(270));
            backupFromBackdropBlueStacksCenter = new Pose2d(-31.5, 46.5, Math.toRadians(270));

            dropOnBackdropRedLeftStacks = new Pose2d(27.5, 53.5, Math.toRadians(270));
            dropOnBackdropRedRightStacks = new Pose2d(51, 61, Math.toRadians(270));
            dropOnBackdropRedCenterStacks = new Pose2d(42.5, 50.5, Math.toRadians(270));

            dropOnBackdropRedLeftStacksHi = new Pose2d(27.5, 54.5, Math.toRadians(270));
            dropOnBackdropRedRightStacksHi = new Pose2d(51, 62.5, Math.toRadians(270));
            dropOnBackdropRedCenterStacksHi = new Pose2d(44, 52, Math.toRadians(270));

            backAwayFromRedTape = new Pose2d(64.5, -36, Math.toRadians(270));
            backAwayFromBlueTape = new Pose2d(-60, -45, Math.toRadians(270));

            parkBlueBackdropSide = new Pose2d(-64, 46, Math.toRadians(270));
            parkRedBackdropSide = new Pose2d(64, 52, Math.toRadians(245));

            parkBlueStacksSide = new Pose2d(0, 55, Math.toRadians(230));
            parkRedStacksSide = new Pose2d(12, 53, Math.toRadians(270));

            parkRedStacksSideRt = new Pose2d(10, 50, Math.toRadians(270));
            parkRedStacksSideLft = new Pose2d(10, 50, Math.toRadians(270));
        } else if(RobotConstants.bot == RobotConstants.Chassis.B7253) {
        moveAwayFromWallRedBackdrop = new Pose2d(50, 15, Math.toRadians(180));
        moveAwayFromWallBlueBackdrop = new Pose2d(-50, 15, Math.toRadians(0));

        moveAwayFromWallRedStacks = new Pose2d(52, -35, Math.toRadians(180));
        moveAwayFromWallBlueStacks = new Pose2d(-54, -35, Math.toRadians(0));

        moveTowardsRedBackdrop = new Pose2d(62, 20, Math.toRadians(270));
        moveTowardsBlueBackdrop = new Pose2d(-60, 8, Math.toRadians(270));

        moveTowardsRedBackdropLft = new Pose2d(sx*59, 15, flip + sh*Math.toRadians(270));
        byBlueLoadStation = new Pose2d(sx * 59, -48, flip + sh*Math.toRadians(270));

        moveTowardsBlueBackdropHdAdj = new Pose2d(-35, 38, Math.toRadians(245));
        moveTowardsRedBackdropHdAdj = new Pose2d(40, 45, Math.toRadians(295));

        moveTowardsRedBackdropHdAdjLft = new Pose2d(35, 30, Math.toRadians(295));
        moveTowardsRedBackdropHdAdjRt = new Pose2d(30, 45, Math.toRadians(295));

        dropPixelRedRightTapeBackdrop = new Pose2d(sx*34, 20.5, flip + sh*Math.toRadians(-30));
        dropPixelRedCenterTapeBackdrop = new Pose2d(sx*28, 16, flip + sh*Math.toRadians(-1));
        dropPixelRedLeftTapeBackdrop = new Pose2d(sx * 31, 3, flip + sh*Math.toRadians(90));

        dropPixelRedLeftTapeBackdropAdj = new Pose2d(32, 9, Math.toRadians(90));
        dropPixelRedRightTapeBackdropAdj = new Pose2d(36, 27, Math.toRadians(0));

        dropPixelBlueRightTapeBackdrop = new Pose2d(-31, 4, Math.toRadians(90));
        dropPixelBlueCenterTapeBackdrop = new Pose2d(-28, 15, Math.toRadians(180));
        dropPixelBlueLeftTapeBackdrop = new Pose2d(-31, 18, Math.toRadians(270));
            dropPixelBlueLeftTapeBackdrop2 = new Pose2d(-38, 22, Math.toRadians(180));

        dropPixelBlueCenterBackTapeAdj = new Pose2d(-33, 15, Math.toRadians(-180));
        dropPixelRedCenterBackTapeAdj = new Pose2d(32, 18, Math.toRadians(0));

        dropPixelRedRightTapeStacks = new Pose2d(28, -26, Math.toRadians(270));
        dropPixelRedCenterTapeStacks = new Pose2d(18.5, -40, Math.toRadians(270));
//        dropPixelRedLeftTapeStacks = new Pose2d(32, -45, Math.toRadians(0));
        dropPixelRedLeftTapeStacks = new Pose2d(26, -49, Math.toRadians(270));

        dropPixelRedRightTapeStacksAdj = new Pose2d(32, -46, Math.toRadians(270));

        dropPixelRedCenterStkTapeAdj = new Pose2d(26, -45, Math.toRadians(270));
        dropPixelBlueCenterStkTapeAdj = new Pose2d(-27, -46, Math.toRadians(270));

        dropPixelBlueRightTapeStacks = new Pose2d(-26, -49, Math.toRadians(270));
        dropPixelBlueCenterTapeStacks = new Pose2d(-19.5, -41, Math.toRadians(270));
        dropPixelBlueLeftTapeStacks = new Pose2d(-28.5, -26.75, Math.toRadians(270));

        moveFromBlueRightTapeStacks = new Pose2d(-35, -57, Math.toRadians(270));
        moveFromBlueLeftTapeStacks = new Pose2d(-32, -44, Math.toRadians(270));
        moveFromBlueCenterTapeStacks = new Pose2d(-24, -52, Math.toRadians(270));

        moveFromRedLeftTapeStacks = new Pose2d(33.5, -58, Math.toRadians(270));
        moveFromRedRightTapeStacks = new Pose2d(32, -40, Math.toRadians(270));
        moveFromRedCenterTapeStacks = new Pose2d(24, -52, Math.toRadians(230));

        dropOnBackdropBlueLeftBackdropHi = new Pose2d(-43, 55.5, Math.toRadians(270));
        dropOnBackdropBlueRightBackdropHi = new Pose2d(-25, 55.5, Math.toRadians(270));
        dropOnBackdropBlueCenterBackdropHi = new Pose2d(-37, 55.5, Math.toRadians(270));

        dropOnBackdropRedLeftBackdrop = new Pose2d(sx*24, 56, flip + sh*Math.toRadians(270 -.01));
        dropOnBackdropRedRightBackdrop = new Pose2d(sx*41, 64, flip + sh*Math.toRadians(270));
        dropOnBackdropRedCenterBackdrop = new Pose2d(sx*30.25, 63.5, flip + sh*Math.toRadians(270));

        dropOnBackdropBlueRightBackdrop = new Pose2d(sx*26.25, 54.75, flip + sh*Math.toRadians(269.99));
        dropOnBackdropBlueLeftBackdrop = new Pose2d(sx*39.5, 63, flip + sh*Math.toRadians(270));
        dropOnBackdropBlueCenterBackdrop = new Pose2d(sx*30.25, 60, flip + sh*Math.toRadians(270));

        moveAwayFromCRedBackdropTape = new Pose2d(38, 15, Math.toRadians(0));
        moveAwayFromLRedBackdropTape = new Pose2d(32, 15, Math.toRadians(90));
        moveAwayFromRRedBackdropTape = new Pose2d(41, 24, Math.toRadians(0));

        moveAwayFromCBlueBackdropTape = new Pose2d(-45, 15, Math.toRadians(180));
        moveAwayFromLBlueBackdropTapeAdj = new Pose2d(-33, 14, Math.toRadians(270));
        moveAwayFromRBlueBackdropTape = new Pose2d(-32, 15, Math.toRadians(90));

        moveAwayFromLBlueBackdropTape = new Pose2d(-49, 14, Math.toRadians(270));

        reverseFromRedBackdropBk = new Pose2d(36, 38, Math.toRadians(270));
        reverseFromBlueBackdropBk = new Pose2d(-36, 42, Math.toRadians(270));

        reverseFromRedBackdropStk = new Pose2d(36, 42, Math.toRadians(310));
        reverseFromBlueBackdropStk = new Pose2d(-36, 42, Math.toRadians(230));

        dropOnBackdropBlueLeftStacks = new Pose2d(-42.5, 54.75, Math.toRadians(270));
        dropOnBackdropBlueRightStacks = new Pose2d(-32, 50.5, Math.toRadians(270));
        dropOnBackdropBlueCenterStacks = new Pose2d(-36, 54, Math.toRadians(270));

        dropOnBackdropBlueLeftStacksHi = new Pose2d(-42, 59, Math.toRadians(270));
        dropOnBackdropBlueRightStacksHi = new Pose2d(-27.5, 51.5, Math.toRadians(270));
        dropOnBackdropBlueCenterStacksHi = new Pose2d(-40.75, 58.5, Math.toRadians(270));
        backupFromBackdropBlueStacksCenter = new Pose2d(-31.5, 46.5, Math.toRadians(270));

        dropOnBackdropRedLeftStacks = new Pose2d(sx*30, 53.5, flip + sh*Math.toRadians(270));
        dropOnBackdropRedRightStacks = new Pose2d(46, 54, Math.toRadians(270));
        dropOnBackdropRedCenterStacks = new Pose2d(37, 48.5, Math.toRadians(270));

        dropOnBackdropRedLeftStacksHi = new Pose2d(29, 56, Math.toRadians(270));
        dropOnBackdropRedRightStacksHi = new Pose2d(41.5, 60, Math.toRadians(270));
        dropOnBackdropRedCenterStacksHi = new Pose2d(39, 56.5, Math.toRadians(270));

        backAwayFromRedTape = new Pose2d(64.5, -36, Math.toRadians(270));
        backAwayFromBlueTape = new Pose2d(-64, -37.5, Math.toRadians(270));

        parkBlueBackdropSide = new Pose2d(-64, 46, Math.toRadians(270));
        parkRedBackdropSide = new Pose2d(64, 52, Math.toRadians(245));

        parkBlueStacksSide = new Pose2d(0, 55, Math.toRadians(230));
        parkRedStacksSide = new Pose2d(12, 53, Math.toRadians(270));

        parkRedStacksSideRt = new Pose2d(10, 50, Math.toRadians(270));
        parkRedStacksSideLft = new Pose2d(10, 50, Math.toRadians(270));
    }
        /* End Qualifier Route Points */

        //2023 tangents
        ninetyFive = flip + sh*Math.toRadians(95);
        ninety = flip + sh*Math.toRadians(90);
        seventy = flip + sh*Math.toRadians(70);
        sixty = flip + sh*Math.toRadians(60);
        oneFifteen = flip + sh*Math.toRadians(115);
        twotwentyfive = flip + sh*Math.toRadians(225);
        twotwenty = flip + sh*Math.toRadians(220);
        twosixty = flip + sh*Math.toRadians(260);
        oneten = flip + sh*Math.toRadians(110);
        twofifity = flip + sh*Math.toRadians(250);
        oneeighty = flip + sh*Math.toRadians(180);
        thirtyfive = flip + sh*Math.toRadians(35);
        thirty = flip + sh*Math.toRadians(30);
        ten = flip + sh*Math.toRadians(10);
        oneHundred = flip + sh*Math.toRadians(100);
        twoTen = flip + sh*Math.toRadians(210);
        forty = flip + sh*Math.toRadians(40);
        zero = flip + sh*Math.toRadians(0);
        onesixtyfive = flip + sh*Math.toRadians(165);
        fifteen = flip + sh*Math.toRadians(15);
        threetwenty = flip + sh*Math.toRadians(320);
        oneNinety = flip + sh*Math.toRadians(190);
        eightyfive = flip + sh*Math.toRadians(85);
        oneforty = flip + sh*Math.toRadians(140);
        threeFifty = flip + sh*Math.toRadians(350);
        sixtyfive = flip + sh*Math.toRadians(65);
        onefifity = flip + sh*Math.toRadians(150);
        twoseventy = flip + sh*Math.toRadians(270);
        twoninety = flip + sh*Math.toRadians(290);
        twoeighty = flip + sh*Math.toRadians(280);
        fiftyfive = flip + sh*Math.toRadians(55);
        threefortyfive = flip + sh*Math.toRadians(345);
     }

     protected double botLen;
     protected double botWid;
     protected double botBackToCtr;
     protected double botSideToCtr;
     protected double hubRad;

     protected double strtX;
     protected double strtY;
     protected double strtH;

     protected Pose2d moveFromStart;
     protected Pose2d turnBox;
     protected Pose2d coneDropSECAJUST;
	 //public final Pose2d Start;
     protected Pose2d awayFromStart;
     protected Pose2d awayFromStart2;
     protected Pose2d toTurnBox;
     //protected final Pose2d seven;
     //protected final Pose2d six;
    protected Pose2d red_side;
    protected Pose2d center;
    protected Pose2d top;

   protected Pose2d middle;

    protected Pose2d bottom;
     protected Pose2d staring_point;
     protected Pose2d team_element;
     protected Pose2d white_pixel_middle;
     protected Pose2d back_drop;
     protected Pose2d startCSRedHigh;
     protected Pose2d startSpecimenSide;
     protected Pose2d specimen1;
     protected Pose2d specimen1PosPosToPush;
     protected Pose2d specimen1PosToPush;
     protected Pose2d dropPreSpecimen;
    protected Pose2d corner;
    protected Pose2d corner2;
    protected Pose2d subPos;
    protected Pose2d subPos2;
    protected Pose2d subPos3;
    protected Pose2d corner3;
    protected Pose2d preSubPos;
     protected Pose2d startSample;
     protected Pose2d startSmallTri;
    protected Pose2d simpleShootFar;

    protected Pose2d startSmallTriBlue;
    protected Pose2d humanZone;
    protected Pose2d shootFarPosBLUE;
    protected Pose2d shootFarfarPosBLUE;
    protected Pose2d PrepPosIntake1;
    protected Pose2d PrepPosIntake1BLUE;

    protected Pose2d intookpos1;
    protected Pose2d intookpos1BLUE;
    protected Pose2d intookpos1BLUEDONE;

    protected Pose2d intakepos2;
    protected Pose2d intakepos2B;
    protected Pose2d intakeloading2;
    protected Pose2d intakeloading2B;
    protected Pose2d intakepos3;
    protected Pose2d intakepos3B;
    protected Pose2d intakeloading3;
    protected Pose2d intakeloading3B;
    protected Pose2d shootPos3;
    protected Pose2d intakepos4;
    protected Pose2d intakeloading4;
    protected Pose2d shootNearPos;
    protected Pose2d shootNearPosB;

     protected Pose2d preleverwiffle;
     protected Pose2d intakewifflelever;
     protected Pose2d prehitlever;
    protected Pose2d hitlever;

    protected Pose2d prenearpos;
     protected Pose2d nearpos;
     protected Pose2d shootFarPos;

    protected Pose2d shootFarPosB;
     protected Pose2d pregotoparkwiffle;
     protected Pose2d  gotoparkwiffle;
     protected Pose2d pregotogoalwiffles;
     protected Pose2d gotogoalwiffles;
     protected Pose2d parkInside1Red;
     protected Pose2d parkInside2Red;
     protected Pose2d parkOutside1Red;
     protected Pose2d parkOutside2Red;
     protected Pose2d parkInside1Blue;
     protected Pose2d parkInside2Blue;
     protected Pose2d parkOutside1Blue;
    protected Pose2d parkOutside2BLUE;
    protected Pose2d tempParkRed;
    protected Pose2d tempParkBlue;
    protected Pose2d startSpecimen;
     protected Pose2d purplePixelPlaceCenterTop;
    protected Pose2d dropCenterPixel;
    protected Pose2d hangSpecimen;
    protected Pose2d moveBackFromSpecimen;
    protected Pose2d sample1;
    protected Pose2d redWall;
    protected Pose2d blueFar;
    protected Pose2d shootPreWhiffles;
    protected Pose2d blueWall;
    protected Pose2d aroundPartner;
    protected Pose2d shootZonefornear;
    protected Pose2d shootZonefornearblue;

    protected Pose2d aroundPartnerBlue;
    protected Pose2d intakeGoal;
    protected Pose2d intakeGoalBlue;
    protected Pose2d intakeGoalWhiffles;
    protected Pose2d shootGoalWhiffles;
    protected Pose2d shootGoalWhifflesinside;
    protected Pose2d route5movementfromstart;


    protected Pose2d helpCollectLever5bluefar;


    protected Pose2d helpCollectLever5blue;
    protected Pose2d moveToLeverblue;



    protected Pose2d intakeGate;
    protected Pose2d intakeGateBlue;
    protected Pose2d intakeGateWhiffles;
    protected Pose2d intakeGateWhifflesBlue;
    protected Pose2d intakeGoalWhifflesBlue;
    protected Pose2d shootGateWhiffles;
    protected Pose2d shootGateWhifflesBlue;
    protected Pose2d shootGateWhifflesBluebackup;
    protected Pose2d aroundWhiffles;
    protected Pose2d shootGoalWhifflesBlue;
    protected Pose2d intakePark;
    protected Pose2d intakeParkBlue;
    protected Pose2d intakeParkWhiffles;
    protected Pose2d intakeParkWhifflesBlue;
    protected Pose2d shootParkWhiffles;
    protected Pose2d shootParkWhifflesBlue;
    protected Pose2d redGoal;
    protected Pose2d blueGoal;
    protected Pose2d sample2;
    protected Pose2d rotateToSample2;
    protected Pose2d sample3;
    protected Pose2d positionSample3;
    protected Pose2d turnStraight;
    protected Pose2d positionToPark;
    protected Pose2d park;
    protected Pose2d park2;

    protected Pose2d deliverSample3ToBasket;
    protected Pose2d deliverSample2ToBasket;
    protected Pose2d deliverSampleToBasket;
    protected Pose2d dropCenterPixelAdj;
    protected Pose2d dropCenterPixelStacksAdj;
    protected Pose2d dropCenterPixelLeftAdj;
    protected Pose2d dropCenterPixelCenterAdjBlue;
    protected Pose2d dropCenterPixelLeftAdjBlue;
    protected Pose2d dropCenterPixelRightAdjBlue;
    protected Pose2d dropCenterPixelLeftAdjRed;
    protected Pose2d dropCenterPixelBackwards;
    protected Pose2d dropLeftPixelBackwardsBlue;
    protected Pose2d dropRightPixelBackwardsRed;
    protected Pose2d dropRightPixelBackwardsBlue;
    protected Pose2d dropLeftPixelBackwardsRed;
    protected Pose2d dropCenterPixelBackwardsBlue;
    protected Pose2d dropCenterPixelBackwardsRed;
    protected Pose2d dropCenterPixelBackwardsRedBd;
    protected Pose2d dropRightPixelBackwardsRedBd;
    protected Pose2d dropLeftPixelBackwardsRedBd;
    protected Pose2d dropCenterPixelBackwardsLeftBackdrop;
    protected Pose2d whatchamacallit;
    protected Pose2d pickUpPixelStackLeft;
    protected Pose2d pickUpPixelStackLeftBlue;
    protected Pose2d pickUpPixelStackLeftLeft;
    protected Pose2d pickUpPixelStackLeftCenter;
    protected Pose2d pickUpPixelStackCenter;
    protected Pose2d pickUpPixelStackCenterStacks;
    protected Pose2d pickUpPixelStackCenterBlue;
    protected Pose2d pickUpPixelStackRight;
    protected Pose2d pickUpPixelStackRightCircuit;
    protected Pose2d pickUpPixelStackRightRedLeft;
    protected Pose2d pickUpPixelStackRightCircuitBlue;
    protected Pose2d pickUpPixelStackRightCircuitBlueCenter;
    protected Pose2d pickUpPixelStackRightCircuitBlueRight;
    protected Pose2d pickUpPixelStackRightLeft;
    protected Pose2d dropLeftPixel;
    protected Pose2d getDropRightPixelBlue;
    protected Pose2d startRedFar;
    protected Pose2d startRedWall;
    protected Pose2d shootfaronred;
    protected Pose2d shootfaronredpre;
    protected Pose2d shootfarfaronred;
    protected Pose2d shootfarfarfaronred;
    protected Pose2d shootfarfarfarfaronred;

    protected Pose2d startBlueFar;
    protected Pose2d moveToPark;
    protected Pose2d collect2;
    protected Pose2d helpcollect2;
    protected Pose2d helpcollect3;
    protected Pose2d helpcollect4;
    protected Pose2d helpcollect5;
    protected  Pose2d moveToLever;
    protected  Pose2d collect3;
    protected  Pose2d helpCollectLever2;
    protected  Pose2d helpCollectLever3;
    protected  Pose2d helpCollectLever4;
    protected  Pose2d helpCollectLever5;

    protected Pose2d moveToParkBlue;
    protected Pose2d collect2Blue;
    protected Pose2d helpcollect2Blue;
    protected Pose2d helpcollect3Blue;
    protected Pose2d helpcollect4Blue;
    protected Pose2d helpcollect5Blue;

    protected Pose2d nearLeaver;
    protected Pose2d shootNear;
    protected Pose2d shootNearForSpline;
    protected Pose2d startBlueWall;
    protected Pose2d shootNearBlue;
    protected Pose2d goNear2Blue;
    protected Pose2d goNear3Blue;
    protected Pose2d goNear1Blue;
    protected Pose2d shootNearBlueSpline;
    protected Pose2d collect1Blue;
    protected Pose2d collect3Blue;
    protected Pose2d moveToHumanPlayerZone;
    protected Pose2d collect1;
    protected Pose2d shootNear2;

    protected Pose2d dropRightPixel;
    protected Pose2d dropRightPixelCircuit;
    protected Pose2d dropRightPixelCuircut;
    protected Pose2d dropLeftPixelBlue;
    protected Pose2d dropRightPixelBlue;
    protected Pose2d backAwayBackDrop;
    protected Pose2d underRigging;
    protected Pose2d pointForRobotToPixelStack;
    protected Pose2d pointForRobotToPixelStackBlue;
    protected Pose2d aboveRigging;
    protected Pose2d aboveRiggingBackwards;
    protected Pose2d underCenter;

    protected Pose2d pointcuzwhynot;
    protected Pose2d underdoor;
    protected Pose2d startCSRedLow;
    protected Pose2d parked;
    protected Pose2d startCBlueHigh;
    protected Pose2d startCSBlueLow;
    protected Pose2d underMiddlePoint;
    protected Pose2d parkDoor;
    protected Pose2d parkWall;
    protected Pose2d teamElementLeft;
    protected Pose2d teamElementRight;
    protected Pose2d teamElementCenter;
    protected Pose2d teamElementCenterLow;
    protected Pose2d teamElementLeftLow;
    protected Pose2d teamElementRightLow;

    protected Pose2d extraPointHigh;
    protected Pose2d extraPointLow;



    /* Qualifier Route Points */
    protected Pose2d moveAwayFromWallRedBackdrop;
    protected Pose2d moveAwayFromWallBlueBackdrop;

    protected Pose2d moveAwayFromWallRedStacks;
    protected Pose2d moveAwayFromWallBlueStacks;

    protected Pose2d moveTowardsRedBackdrop;
    protected Pose2d moveTowardsBlueBackdrop;

    protected Pose2d moveTowardsRedBackdropLft;
    protected Pose2d byBlueLoadStation;

    protected Pose2d moveTowardsBlueBackdropHdAdj;
    protected Pose2d moveTowardsRedBackdropHdAdj;

    protected Pose2d moveTowardsRedBackdropHdAdjLft;
    protected Pose2d moveTowardsRedBackdropHdAdjRt;

    protected Pose2d dropPixelRedRightTapeBackdrop;
    protected Pose2d dropPixelRedCenterTapeBackdrop;
    protected Pose2d dropPixelRedLeftTapeBackdrop;

    protected Pose2d dropPixelRedLeftTapeBackdropAdj;
    protected Pose2d dropPixelRedRightTapeBackdropAdj;

    protected Pose2d dropPixelBlueRightTapeBackdrop;
    protected Pose2d dropPixelBlueCenterTapeBackdrop;
    protected Pose2d dropPixelBlueLeftTapeBackdrop;
    protected Pose2d dropPixelBlueLeftTapeBackdrop2;

    protected Pose2d dropPixelRedRightTapeStacks;
    protected Pose2d dropPixelRedCenterTapeStacks;

    protected Pose2d dropPixelRedRightTapeStacksAdj;

    protected Pose2d dropPixelRedCenterStkTapeAdj;
    protected Pose2d dropPixelBlueCenterStkTapeAdj;

    protected Pose2d dropPixelRedCenterBackTapeAdj;
    protected Pose2d dropPixelRedLeftBackTapeAdj;
    protected Pose2d dropPixelBlueCenterBackTapeAdj;

    protected Pose2d dropPixelBlueCenterTapeStacks;
    protected Pose2d dropPixelBlueLeftTapeStacks;

    protected Pose2d moveFromBlueRightTapeStacks;
    protected Pose2d moveFromBlueLeftTapeStacks;
    protected Pose2d moveFromBlueCenterTapeStacks;

    protected Pose2d moveFromRedRightTapeStacks;
    protected Pose2d moveFromRedLeftTapeStacks;
    protected Pose2d moveFromRedCenterTapeStacks;

    protected Pose2d dropOnBackdropBlueLeftBackdrop;
    protected Pose2d dropOnBackdropBlueRightBackdrop;
    protected Pose2d dropOnBackdropBlueCenterBackdrop;

    protected Pose2d dropOnBackdropBlueLeftBackdropHi;
    protected Pose2d dropOnBackdropBlueRightBackdropHi;
    protected Pose2d dropOnBackdropBlueCenterBackdropHi;

    protected Pose2d dropOnBackdropRedLeftBackdrop;
    protected Pose2d dropOnBackdropRedRightBackdrop;
    protected Pose2d dropOnBackdropRedCenterBackdrop;

    protected Pose2d moveAwayFromCRedBackdropTape;
    protected Pose2d moveAwayFromLRedBackdropTape;
    protected Pose2d moveAwayFromRRedBackdropTape;

    protected Pose2d moveAwayFromCBlueBackdropTape;
    protected Pose2d moveAwayFromLBlueBackdropTapeAdj;
    protected Pose2d moveAwayFromRBlueBackdropTape;

    protected Pose2d moveAwayFromLBlueBackdropTape;

    protected Pose2d reverseFromRedBackdropBk;
    protected Pose2d reverseFromBlueBackdropBk;

    protected Pose2d reverseFromRedBackdropStk;
    protected Pose2d reverseFromBlueBackdropStk;

    protected Pose2d dropOnBackdropBlueLeftStacks;
    protected Pose2d dropOnBackdropBlueRightStacks;
    protected Pose2d dropOnBackdropBlueCenterStacks;

    protected Pose2d dropOnBackdropBlueLeftStacksHi;
    protected Pose2d dropOnBackdropBlueRightStacksHi;
    protected Pose2d dropOnBackdropBlueCenterStacksHi;
    protected Pose2d backupFromBackdropBlueStacksCenter;

    protected Pose2d dropOnBackdropRedLeftStacks;
    protected Pose2d dropOnBackdropRedRightStacks;
    protected Pose2d dropOnBackdropRedCenterStacks;

    protected Pose2d dropOnBackdropRedLeftStacksHi;
    protected Pose2d dropOnBackdropRedRightStacksHi;
    protected Pose2d dropOnBackdropRedCenterStacksHi;

    protected Pose2d backAwayFromRedTape;
    protected Pose2d backAwayFromBlueTape;

    protected Pose2d parkBlueBackdropSide;
    protected Pose2d parkRedBackdropSide;
    protected Pose2d parkBlueStacksSide;
    protected Pose2d parkRedStacksSide;

    protected Pose2d parkRedStacksSideLft;
    protected Pose2d parkRedStacksSideRt;

    protected Pose2d dropPixelRedLeftTapeStacks;
    protected Pose2d dropPixelBlueRightTapeStacks;

    /* End Qualifier Route Points */




     /* States Competition Points */
     public Pose2d start;
     protected Pose2d moveAwayFromStart;
     public Pose2d ajustForWheelHittingWall;
     protected Pose2d coneDropLowJunct;

    protected Pose2d start2;

    protected Pose2d specimen2;

    protected Pose2d specimen3;

    protected Pose2d specimenPark1;
    protected Pose2d waitForHumanPlayer;
    protected Pose2d waitForHumanPlayer2;
    protected Pose2d waitForHumanPlayer3;



    protected Pose2d forward;
     protected Pose2d fourPtJunctDropSpline;
     protected Pose2d fourPtJunctDropSplineAJUSTED;



     /* Parks */
     protected Pose2d parkCENTER;
     protected Pose2d parkRIGHT;
     protected Pose2d parkLEFT;

  protected Pose2d starting_point_top;

     /* Tangents */
     protected double moveAwayFromStartTangent;
     protected double fourPtJnctDropTangent;
     protected double reverseFourPtJunctDropTangent;
     protected double sharedHighJunctionDropSplineTangent;

     protected double sharedHighJunctionDropReverseToConstackTangent;
     protected double reverseHighJunctionDropSplineTangent;
     protected double oppositionHighJunctionDropSplineTangent;
     protected double reverseOppositionhighJunctionDropSplineTangent;
     protected double thelowtanjentreverse;
    protected double ninetyFive;
    protected double ninety;
    protected double seventy;
    protected double sixty;
    protected double oneFifteen;
    protected double twofifity;
    protected double oneeighty;
    protected double thirtyfive;
    protected double twotwentyfive;
    protected double twotwenty;
    protected double twosixty;
    protected double oneten;
    protected double oneHundred;
    protected double thirty;
    protected double ten;
    protected double twoTen;
    protected double forty;
    protected double oneforty;
    protected double zero;
    protected double onesixtyfive;
    protected double fifteen;
    protected double threetwenty;
    protected double eightyfive;
    protected double oneNinety;
    protected double threeFifty;
    protected double sixtyfive;
    protected double fiftyfive;
    protected double threefortyfive;
    protected double onefifity;
    protected double twoseventy;
    protected double twoninety;
    protected double twoeighty;

     protected double sharedHighJunctionDropSplineHEADING;
     protected double coneStackGrabHFromSharedHighJunctionHEADING;
}
