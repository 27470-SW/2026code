package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Field.Start_Pos.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/* ==============================================================================================
 * Decode_Route: picks which route to build, and lists every function a route can call.
 *
 * Shared by TeamCode and MeepMeepTesting.  It is abstract: each platform makes its own
 * subclass that says what the functions DO:
 *   - TeamCode:        RobotRoute     (runs the shooter, intake, ...)
 *   - MeepMeepTesting: MeepMeepRoute  (prints what would happen)
 *
 * To add a new function that routes can use (route.addFunction(route::myFunction)):
 *   1. add  protected abstract void myFunction();  in the list below
 *   2. write it in TeamCode's RobotRoute
 *   3. write it in MeepMeepTesting's MeepMeepRoute (usually just say("myFunction"))
 * The compiler will tell you if you forget step 2 or 3.
 * ============================================================================================== */
public abstract class Decode_Route extends Decode_Poses
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
   * RobotRoute (TeamCode) and MeepMeepRoute (MeepMeepTesting) each fill these in.
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

  /** Show the route choices on the driver station (RobotRoute does this; MeepMeep doesn't). */
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
       RouteLog.dd(TAG, "in InitTrajectories2");
       dashboardConfig();

       Pose2d lastPose;

       RouteLog.dd(TAG, "choosing park");

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
       else if( startPos == START_FAR_RED6)
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
       RouteLog.dd(TAG, "Park choice %s",parkPos);

       RouteLog.dd(TAG, "route made");

/*
       RouteLog.dd(TAG, "making park");
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
       RouteLog.dd(TAG, "park made");

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
}
