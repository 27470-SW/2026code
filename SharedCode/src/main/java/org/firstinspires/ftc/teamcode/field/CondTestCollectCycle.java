package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;

/* Conditionals test route, state 6 (the whileBlock body): drive down the row collecting,
 * then come back.  A lap must end where the whileBlock started (58, -24, 270). */
public class CondTestCollectCycle {
    Decode_Route route;

    public CondTestCollectCycle(Decode_Route constructorRoute) {
        route = constructorRoute;
    }

    public void makeTraj() {
        route.addFunction(route::intakeOn);
        route.addLocation(route.pointForRobotToPixelStack, LINE, HEAD_LINEAR);       // (58, -44, 270)
        route.addEvent(Route.Action.WAIT, 0.5);
        route.addFunction(route::intakeOff);
        route.addLocation(route.pointForRobotToPixelStackBlue, LINE, HEAD_LINEAR);   // back to (58, -24, 270)
    }
}
