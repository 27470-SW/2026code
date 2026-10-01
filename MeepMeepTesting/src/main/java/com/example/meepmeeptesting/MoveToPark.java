package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Route.Heading.HEAD_LINEAR;
import static com.example.meepmeeptesting.Route.Movement.LINE;
import static com.example.meepmeeptesting.Route.Movement.SPLINE;
import static com.example.meepmeeptesting.Field.Highways.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MoveToPark {
    Route route;
    public MoveToPark(Route constructorRoute) {
        route = constructorRoute;
   }

    public void makeTraj(Field.Highways parkPos, Field.Alliance alliance) {
        if(parkPos == WALL){
//            route.addMovement(TURN, 0.5);
            route.addEvent(Route.Action.TANGENT, route.threetwenty);
            route.addLocation(route.parkWall,SPLINE,HEAD_LINEAR,route.eightyfive);
            route.addFunction(route::armDropSpikePos);
        }
        if(parkPos == Park1){
            route.addEvent(Route.Action.TANGENT, route.threeFifty);
            route.addLocation(route.parkDoor,SPLINE,HEAD_LINEAR, route.sixtyfive);
            route.addFunction(route::armDropSpikePos);
        }
        else{

        }
        if(parkPos != Park2){
                Pose2d backedUp = new Pose2d(route.getLastPose().getX(), route.getLastPose().getY()+16, route.getLastPose().getHeading());
                route.addLocation(backedUp, LINE, HEAD_LINEAR);
        }
    }

    public final static int INIT_CONE_STACK = 5;
    public static int conestackNum = INIT_CONE_STACK;

    protected void openClaw()
    {
        System.out.println("OPEN Claw");

    }

    /* Grab the cone in the stack or in the terminal */
    protected void closeClaw()
    {
        System.out.println("Close Claw");

    }


   /*protected final int sx;
    protected final int sy;
    protected int sh;
    protected int sf;
    protected int sr;
    protected final double flip;
    protected final double strtX;
    protected final double strtY;
    protected final double strtH;
    protected Field.Alliance alliance;
    protected PositionOption startPos;
    protected final double botBackToCtr;
    protected final double botSideToCtr;
    double rightSideLineUpToBorderAdjustment = 0.5;

    protected Pose2d moveFromStart;
    protected Pose2d start;*/

}


