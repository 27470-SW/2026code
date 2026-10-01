package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Field.Highways.*;
import static com.example.meepmeeptesting.Route.Heading.*;
import static com.example.meepmeeptesting.Route.Movement.*;

public class MoveToBackdropFromPixelStack {
    Route route;
    public MoveToBackdropFromPixelStack(Route constructorRoute) {
route = constructorRoute;
    }

    public void makeTraj(Field.Highways highway, Field.Highways pixelStack, Route.TeamElement dropLocation, Field.Alliance alliance) {
//       firstMovement(highway);
       secondOption(highway, pixelStack,dropLocation, alliance);
    }




    protected void firstMovement(Field.Highways highway)
    {


        if (highway == Park1) {
            route.addEvent(Route.Action.TANGENT, Math.toRadians(90));
            route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR, Math.toRadians(90));
            route.addLocation(route.dropCenterPixel,SPLINE, HEAD_SPLINE, Math.toRadians(30));

        }
//
        if (highway == WALL) {
            //route.addEvent(Route.Action.TANGENT, Math.toRadians(135));
            route.addLocation(route.underRigging, LINE, HEAD_LINEAR, Math.toRadians(180));

            route.addLocation(route.aboveRigging,LINE,HEAD_LINEAR);
            route.addFunction(route::armToDrop, .2);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(90));
            route.addLocation(route.dropCenterPixel,SPLINE, HEAD_SPLINE, Math.toRadians(160));
            //route.addEvent(Route.Action.TANGENT, Math.toRadians(45));
        }

        if (highway == Park2) {
            route.addLocation(route.underCenter, LINE, HEAD_LINEAR);
            //route.addLocation(route.pointcuzwhynot, SPLINE, HEAD_LINEAR);
            route.addLocation(route.dropCenterPixel,SPLINE, HEAD_SPLINE);
        }

        //route.addEvent(Route.Action.TANGENT, Math.toRadians(90));


    }


    protected void secondOption(Field.Highways highway, Field.Highways pixelStack, Route.TeamElement dropLocation, Field.Alliance alliance){
        if (highway == WALL && pixelStack == WALL){
            //Wall to wall
//            route.addLocation(route.pickUpPixelStackRight, LINE, HEAD_LINEAR);
            route.addEvent(Route.Action.TANGENT, route.sixtyfive);
            route.addFunction(route::armDropSpikePos, 1);
            route.addLocation(route.underRigging, SPLINE, HEAD_LINEAR,route.sixtyfive);
            route.addEvent(Route.Action.TANGENT, route.eightyfive);
            route.addFunction(route::armToDropHigher,2);
            route.addLocation(route.dropCenterPixelBackwards,SPLINE,HEAD_LINEAR, route.oneFifteen);
        }
        if (highway == Park2 && pixelStack == WALL){
            //wall to center
//            route.addLocation(route.pickUpPixelStackRight, LINE, HEAD_LINEAR);
            route.addFunction(route::armDropSpikePos, 1);
            route.addLocation(route.dropCenterPixelBackwards,LINE,HEAD_LINEAR);
            route.addFunction(route::armToDropHigher,2);
        }
        if (highway == Park1 && pixelStack == WALL){
            //wall to door
//            route.addLocation(route.pickUpPixelStackRight, LINE, HEAD_LINEAR);
            route.addEvent(Route.Action.TANGENT, route.oneFifteen);
            route.addFunction(route::armDropSpikePos, 1);
            route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR,route.twoseventy);
            route.addEvent(Route.Action.TANGENT, route.oneHundred);
            route.addFunction(route::armToDropHigher, 2);
//            route.addLocation(route.dropCenterPixelBackwards, SPLINE, HEAD_LINEAR,route.zero);
            route.addLocation(route.dropCenterPixelBackwardsLeftBackdrop, SPLINE, HEAD_LINEAR,route.zero);

        }
        if (highway == Park2 && pixelStack == Park2){
            //center to wall
//            route.addLocation(route.pickUpPixelStackCenter,LINE,HEAD_LINEAR);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(35));
            route.addFunction(route::armDropSpikePos, 1);
            route.addLocation(route.underMiddlePoint,SPLINE,HEAD_LINEAR,Math.toRadians(90));
            route.addFunction(route::armToDropHigher,2);
            route.addLocation(route.dropCenterPixelBackwards,SPLINE,HEAD_LINEAR,Math.toRadians(90));
        }
        if (highway == WALL && pixelStack == Park2){
            //center to Wall
//            route.addLocation(route.pickUpPixelStackCenter, LINE, HEAD_LINEAR);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(0));
            route.addFunction(route::armDropSpikePos, 1);
            route.addLocation(route.underRigging, SPLINE, HEAD_LINEAR,Math.toRadians(0));
            route.addEvent(Route.Action.TANGENT, Math.toRadians(90));
            route.addFunction(route::armToDropHigher,2);
            route.addLocation(route.dropCenterPixelBackwards,SPLINE,HEAD_LINEAR, Math.toRadians(130));
        }
        if (highway == Park1 && pixelStack == Park2){
            //center to door
//            route.addLocation(route.pickUpPixelStackCenter,LINE,HEAD_LINEAR);
            route.addFunction(route::armDropSpikePos, 1);
            route.addLocation(route.underdoor,SPLINE,HEAD_LINEAR,route.oneeighty);
            route.addEvent(Route.Action.TANGENT, route.oneHundred);
            route.addFunction(route::armToDropHigher,2);
            route.addLocation(route.dropCenterPixelBackwards,SPLINE,HEAD_LINEAR,route.thirtyfive);
        }
        if (highway == WALL && pixelStack == Park1){
            //door to wall
//            route.addLocation(route.pickUpPixelStackLeft,LINE,HEAD_LINEAR);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(20));
            route.addFunction(route::armDropSpikePos, 1);
            route.addLocation(route.underRigging, SPLINE, HEAD_LINEAR,Math.toRadians(0));
            route.addEvent(Route.Action.TANGENT, Math.toRadians(90));
            route.addFunction(route::armToDropHigher,2);
            route.addLocation(route.dropCenterPixelBackwards,SPLINE,HEAD_LINEAR,Math.toRadians(120));
        }
        if (highway == Park2 && pixelStack == Park1){
            //door to center
//            route.addLocation(route.pickUpPixelStackLeft,LINE,HEAD_LINEAR);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(30));
            route.addFunction(route::armDropSpikePos, 1);
            route.addLocation(route.underMiddlePoint, SPLINE, HEAD_LINEAR,Math.toRadians(90));
            route.addFunction(route::armToDropHigher,2);
            route.addLocation(route.dropCenterPixelBackwards,SPLINE,HEAD_LINEAR,Math.toRadians(90));
        }
        if (highway == Park1 && pixelStack == Park1){
            //door to door
//            route.addLocation(route.pickUpPixelStackLeft,LINE,HEAD_LINEAR);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(90));
            route.addFunction(route::armDropSpikePos, 1);
            route.addLocation(route.dropCenterPixelBackwards,SPLINE,HEAD_LINEAR,Math.toRadians(30));
            route.addFunction(route::armToDropHigher, 2);
        }

        if(alliance == Field.Alliance.RED) {

            if (dropLocation == Route.TeamElement.RIGHT) {
                route.addLocation(route.dropRightPixelCircuit, LINE, HEAD_LINEAR, Math.toRadians(90));

            } else if (dropLocation == Route.TeamElement.CENTER) {
                route.addLocation(route.dropCenterPixel, LINE, HEAD_LINEAR, Math.toRadians(90));

            } else {
                route.addLocation(route.dropLeftPixel, LINE, HEAD_LINEAR, Math.toRadians(90));

            }
        }else{ //the points below were created for red side, but backdrop points can't be reflected sooooooo right == left, and left == right
            if (dropLocation == Route.TeamElement.RIGHT) {
                route.addLocation(route.dropRightPixelBlue, LINE, HEAD_LINEAR, Math.toRadians(90));

            } else if (dropLocation == Route.TeamElement.CENTER) {
                route.addLocation(route.dropCenterPixel, LINE, HEAD_LINEAR, Math.toRadians(90));

            } else {
                route.addLocation(route.dropLeftPixelBlue, LINE, HEAD_LINEAR, Math.toRadians(90));

            }
        }



//TODO: eventually uncomment this and take out the first "out pixel" in dropontapethenbackdrop in every stack side under the if statement
//        route.addFunction(route::outPixel);
    }


}


