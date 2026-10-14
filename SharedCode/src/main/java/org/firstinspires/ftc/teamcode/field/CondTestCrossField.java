package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;

/* Conditionals test route, state 4: right after the endIf.  The function goes into every
 * copy of the split route, and the addLocation (with its TANGENT) brings them back together. */
public class CondTestCrossField {
    Decode_Route route;

    public CondTestCrossField(Decode_Route constructorRoute) {
        route = constructorRoute;
    }

    public void makeTraj() {
        route.addFunction(route::wheelOff);
        route.addEvent(Route.Action.TANGENT, Math.toRadians(20));
        route.addLocation(route.starting_point_top, SPLINE, HEAD_LINEAR);      // (60, 12, 0)
    }
}
