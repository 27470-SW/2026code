package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Field.Highways.*;
import static com.example.meepmeeptesting.Route.Heading.*;
import static com.example.meepmeeptesting.Route.Movement.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MoveToPixelStackFromBackdrop {
    Route route;

    public MoveToPixelStackFromBackdrop(Route constructorRoute) {
        route = constructorRoute;
    }

    public void makeTraj(Field.Highways highway, Field.Highways pixelStack, Field.Alliance alliance, Route.TeamElement teamElement, PositionOption startPos) {
        if (alliance == Field.Alliance.RED) {
            if (highway == WALL && pixelStack == WALL) {
                if (teamElement == Route.TeamElement.LEFT) {
                    route.addEvent(Route.Action.TANGENT, route.threetwenty);
                    route.addFunction(route::armDropSpikePos, 1.5);
                    route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_CONSTANT, Math.toRadians(260));
                    route.addEvent(Route.Action.TANGENT, route.twotwentyfive);
                    //try 1
                    route.addFunction(route::armToIntake);
                    route.addFunction(route::intakes);
                    route.addLocation(route.pickUpPixelStackRightRedLeft, SPLINE, HEAD_LINEAR);
                    //
                } else if (teamElement == Route.TeamElement.RIGHT && startPos == Field.StartPos.START_SPECIMENS) {
                    route.addEvent(Route.Action.TANGENT, route.threetwenty);
                    route.addFunction(route::armDrivePos, 1.5);
                    route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_CONSTANT);
                    route.addEvent(Route.Action.TANGENT, route.twotwentyfive);
                    //try 1
                    route.addFunction(route::armToIntake);
                    route.addFunction(route::intakes);
                    route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
                    //
                } else {
                    route.addEvent(Route.Action.TANGENT, route.threetwenty);
                    route.addFunction(route::armDrivePos, 1.5);
                    route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_CONSTANT);
                    route.addEvent(Route.Action.TANGENT, route.twotwentyfive);
                    //try 1
                    route.addFunction(route::armToIntake);
                    route.addFunction(route::intakes);
                    route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
                }

            }

            if (highway == WALL && pixelStack == Park2) {

                // route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, route.threefortyfive);
                route.addFunction(route::armDropSpikePos, 1.5);
                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR);
                route.addEvent(Route.Action.TANGENT, route.twotwenty);
                route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR, route.twosixty);
                route.addEvent(Route.Action.TANGENT, route.oneten);
            }
            if (highway == WALL && pixelStack == Park1) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, route.threefortyfive);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
            }
            if (highway == Park2 && pixelStack == WALL) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
            }
            if (highway == Park2 && pixelStack == Park2) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(270));
                route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR, Math.toRadians(200));
            }
            if (highway == Park2 && pixelStack == Park1) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(180));
                route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR, Math.toRadians(200));
            }
            if (highway == Park1 && pixelStack == WALL) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(210));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
            }
            if (highway == Park1 && pixelStack == Park2) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(210));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR);
            }
            if (highway == Park1 && pixelStack == Park1) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(210));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR);
            }

//            route.addFunction(route::armToIntake);
//            route.addEvent(Route.Action.WAIT, .3);
//            route.addFunction(route::intakes);
//            route.addEvent(Route.Action.WAIT, .5);
//            route.addFunction(route::outFrontPixel);
        } else if (alliance == Field.Alliance.BLUE) {
            if (highway == WALL && pixelStack == WALL) {
                route.printLastPose();
//                route.addLocation(route.dropCenterPixel, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.TANGENT, route.threetwenty);
                route.addFunction(route::armDropSpikePos, 1.5);
                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_CONSTANT, route.sixty);
                route.addEvent(Route.Action.TANGENT, route.twotwentyfive);
                //try 1
                route.addFunction(route::armToIntake);
                route.addFunction(route::intakes);
                //
                if (teamElement == Route.TeamElement.RIGHT && startPos == Field.StartPos.START_SAMPLES) {
                    route.addLocation(route.pickUpPixelStackRightCircuitBlueRight, SPLINE, HEAD_LINEAR);
                } else if (teamElement == Route.TeamElement.CENTER && startPos == Field.StartPos.START_SPECIMENS) {
                    route.addLocation(route.pickUpPixelStackRightCircuitBlueCenter, SPLINE, HEAD_LINEAR);
                } else {
                    route.addLocation(route.pickUpPixelStackRightCircuitBlue, SPLINE, HEAD_LINEAR);
                }

            }

            if (highway == WALL && pixelStack == Park2) {

                // route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, route.threefortyfive);
                route.addFunction(route::armDropSpikePos, 1.5);
                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR, Math.toRadians(-45));
                route.addEvent(Route.Action.TANGENT, route.twotwenty);
                route.addLocation(route.pickUpPixelStackCenterBlue, SPLINE, HEAD_LINEAR, route.twosixty);
                route.addEvent(Route.Action.TANGENT, route.oneten);
            }
            if (highway == WALL && pixelStack == Park1) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, route.threefortyfive);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR, Math.toRadians(-45));
                route.addLocation(route.pickUpPixelStackLeftBlue, SPLINE, HEAD_LINEAR);
            }
//            if (highway == WALL && pixelStack == WALL) {
//                RobotLog.dd(RobotLog.TAG, "got where i thought!")
//                route.printLastPose();
////                route.addLocation(route.dropCenterPixel, LINE, HEAD_LINEAR);
//                route.addEvent(Route.Action.TANGENT, Math.toRadians(200));
//                route.addFunction(route::armDrivePos, 1.5);
//                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_CONSTANT);
//                route.addLocation(route.pickUpPixelStackRightCircuit, LINE, HEAD_LINEAR);
//
//            }
//
//            if (highway == WALL && pixelStack == CENTER) {
//
////                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
//                route.addEvent(Route.Action.TANGENT, Math.toRadians(200));
//                route.addFunction(route::armDrivePos, 1.5);
//                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR);
//                route.addEvent(Route.Action.TANGENT, Math.toRadians(220));
//                route.addLocation(route.pickUpPixelStackCenter, LINE, HEAD_LINEAR, Math.toRadians(30));
//                route.addEvent(Route.Action.TANGENT, Math.toRadians(110));
//            }
//            if (highway == WALL && pixelStack == DOOR) {
//
////                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
//                route.addEvent(Route.Action.TANGENT, Math.toRadians(200));
//                route.addFunction(route::armDrivePos, 1.5);
//                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR);
//                route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR);
//            }
            if (highway == Park2 && pixelStack == WALL) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
            }
            if (highway == Park2 && pixelStack == Park2) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(270));
                route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR, Math.toRadians(200));
            }
            if (highway == Park2 && pixelStack == Park1) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(180));
                route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR, Math.toRadians(200));
            }
            if (highway == Park1 && pixelStack == WALL) {
//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(300));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
            }
            if (highway == Park1 && pixelStack == Park2) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(300));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR);
            }
            if (highway == Park1 && pixelStack == Park1) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(300));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR);
            }

        }
        Pose2d backedUp = new Pose2d(route.getLastPose().getX(), route.getLastPose().getY() + 1.2, route.getLastPose().getHeading());
        route.addLocation(backedUp, LINE, HEAD_LINEAR);
        route.makeNewTraj();
        route.addFunction(route::outFrontPixel);
    }
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




