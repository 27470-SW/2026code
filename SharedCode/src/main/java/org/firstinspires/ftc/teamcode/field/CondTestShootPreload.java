package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;

/* Conditionals test route, state 1: start against the wall and shoot the preloads.
 * The control flow (waitUntil, ifBlock, ...) is in Decode_Route.initTrajectories2. */
public class CondTestShootPreload {
    Decode_Route route;

    public CondTestShootPreload(Decode_Route constructorRoute) {
        route = constructorRoute;
    }

    public void makeTraj() {
        route.addLocation(route.pickUpPixelStackCenter, START, HEAD_LINEAR);   // (10, -62, 270)
        route.addFunction(route::shootFar);         // function right after START: spin up the wheel
        route.addEvent(Route.Action.WAIT, 1.5);
        route.shootMotif(1, route.pickUpPixelStackCenter);
    }
}
