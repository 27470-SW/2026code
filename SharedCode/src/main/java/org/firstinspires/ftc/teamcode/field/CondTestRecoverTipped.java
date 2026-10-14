package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;

/* Conditionals test route, state 3 (elseBlock when the robot is tipped): back off to a
 * different spot and shoot from there.  This branch ends somewhere else than the ifBlock
 * branch, so the route splits until the next addLocation (CondTestCrossField). */
public class CondTestRecoverTipped {
    Decode_Route route;

    public CondTestRecoverTipped(Decode_Route constructorRoute) {
        route = constructorRoute;
    }

    public void makeTraj() {
        route.addFunction(route::intakeOff);
        route.addLocation(route.pointcuzwhynot, LINE, HEAD_LINEAR);            // (36, -37, 270)
        route.addFunction(route::shootFar);
        route.addEvent(Route.Action.WAIT, 1.5);
        route.addFunction(route::shootAllThree);
        route.addEvent(Route.Action.WAIT, 1);
    }
}
