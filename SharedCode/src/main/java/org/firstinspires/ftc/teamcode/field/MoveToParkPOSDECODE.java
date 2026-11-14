package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;

import com.qualcomm.robotcore.util.RobotLog;


public class MoveToParkPOSDECODE {
    String TAG = "PARK_POS_DECODE";
    BioBuzz_Route route;
    private Field.Parks stackToBack;
    private Field.Parks pixelStack;

    private Route.TeamElement teamElement;
    private Field.Alliance alliance;
    public MoveToParkPOSDECODE(BioBuzz_Route constructorRoute) {
        route = constructorRoute;
    }

    public void makeTraj(Field.Park_Pos parkPos, Field.Alliance alliance) {
        RobotLog.dd(TAG, "choosing park in movetoparkposdecode");

        if (parkPos == Field.Park_Pos.PARK1) {
            route.addLocation(route.parkInside1Red, LINE, HEAD_LINEAR);
        } else if (parkPos == Field.Park_Pos.PARK2) {
            route.addLocation(route.parkInside2Red, LINE, HEAD_LINEAR);
        }
         else if (parkPos == Field.Park_Pos.NOPARK) {

            }


        else {
            if(alliance == Field.Alliance.RED)
                route.addLocation(route.tempParkRed, LINE, HEAD_CONSTANT);
            else
                route.addLocation(route.tempParkBlue, LINE, HEAD_CONSTANT);
        }

        RobotLog.dd(TAG, "In movetoparkposdecode Park choice %s",parkPos);







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
}

