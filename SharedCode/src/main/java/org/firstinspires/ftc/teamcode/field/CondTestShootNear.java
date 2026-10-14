package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;

/* Conditionals test route, state 5: drive up to the near shooting spot, shoot, then head
 * to the spike marks to start collecting. */
public class CondTestShootNear {
    Decode_Route route;

    public CondTestShootNear(Decode_Route constructorRoute) {
        route = constructorRoute;
    }

    public void makeTraj() {
        route.addFunction(route::shootWiffleClose);
        route.addLocation(route.dropCenterPixel, LINE, HEAD_LINEAR);           // (10, 60, 90)
        route.shootMotif(1, route.dropCenterPixel);
        route.addFunction(route::wheelOff);
        route.addFunction(route::intakeonandthreeTransitionsDown);
        route.addEvent(Route.Action.TANGENT, Math.toRadians(-30));
        route.addLocation(route.pointForRobotToPixelStackBlue, SPLINE, HEAD_LINEAR);   // (58, -24, 270)
    }
}
