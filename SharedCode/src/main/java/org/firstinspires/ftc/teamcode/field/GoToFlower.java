package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class  GoToFlower {
    BioBuzz_Route route;

    private Route.TeamElement teamElement;
    private Field.Alliance alliance;

    public GoToFlower(BioBuzz_Route constructorRoute) {
        route = constructorRoute;
    }

    public void makeTraj(PositionOption startPos, Field.Park_Pos parkPos, Field.Wiffle_Pos lastLocation) {
  /*
        this.stackToBack = stackToBack;
        if(firstLocation == PIXEL_CENTER){
            pixelStack = Field.Highways.CENTER;
        }else if(firstLocation == PIXEL_WALL){
            pixelStack = WALL;
        }else{
            pixelStack = DOOR;
        }
        this.teamElement = teamElement;
        this.alliance = alliance;
    */
        //  qualifierRoute(startPos,parkPos,firstLocation);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

        route.addLocation(route.startFirst, START, HEAD_LINEAR);
        route.addFunction(route::intakeOn);
        route.addEvent(Route.Action.TANGENT,Math.toRadians(125));
        route.addFunction(route::inclinePlaneDown);
        route.addLocation(route.moveToFlower, SPLINE, HEAD_LINEAR);
        route.addEvent(Route.Action.WAIT, 2.0);
        route.addFunction(route::inclinePlaneUp, 2);
        route.addFunction(route::intakeOff);
        route.addEvent(Route.Action.TANGENT,Math.toRadians(-90));
        route.addLocation(route.shootFlower, LINE, HEAD_LINEAR);
        route.addFunction(route::shootFar);
        route.addEvent(Route.Action.WAIT, 2.0);
        route.addFunction(route::shoot1);

        //Go to flower




    }



    private void viaWall(Pose2d backdrop) {
        route.addLocation(route.byBlueLoadStation, SPLINE, HEAD_LINEAR, route.zero);
        route.makeNewTraj();
        route.addLocation(route.moveTowardsRedBackdropLft, LINE, HEAD_LINEAR, route.ninety);
        route.addFunction(route::armToDrop, 1.5);
        route.addLocation(backdrop, SPLINE, HEAD_LINEAR, route.ninety);
    }

    private void viaCenter(Pose2d backdrop) {
        route.addEvent(Route.Action.TANGENT, route.twoseventy);
        route.addLocation(route.whatchamacallit, SPLINE, HEAD_LINEAR);
        route.addFunction(route::armToDrop, 1.5);
        route.addLocation(backdrop, LINE, HEAD_LINEAR);
    }


   /*rotected final int sx;
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



