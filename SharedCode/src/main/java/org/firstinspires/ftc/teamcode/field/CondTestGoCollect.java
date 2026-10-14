package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;

/* Conditionals test route, state 2: turn the shooter off, start intaking, and drive (slowly)
 * to the collect spot.  Comes right after a waitUntil, so it is the same
 * START -> addFunction -> TANGENT -> SLOW -> addLocation pattern as the very start of a route. */
public class CondTestGoCollect {
    Decode_Route route;

    public CondTestGoCollect(Decode_Route constructorRoute) {
        route = constructorRoute;
    }

    public void makeTraj() {
        route.addFunction(route::wheelOff);
        route.addFunction(route::intakeonandthreeTransitionsDown);
        route.addEvent(Route.Action.TANGENT, Math.toRadians(100));
        route.addEvent(Route.Action.SLOW, 0.5);
        route.addLocation(route.startRedFar, SPLINE, HEAD_LINEAR);             // (20, -60, 300)
        route.addEvent(Route.Action.WAIT, 1);
    }
}
