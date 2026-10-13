package org.firstinspires.ftc.teamcode.field;

/**
 * The few robot settings the shared route code needs (speed limits and robot size).
 *
 * On the robot, TeamCode's RobotConstants.init() copies its values in here, so the robot
 * always uses TeamCode's RobotConstants like before.
 *
 * MeepMeep has no RobotConstants, so it uses the starting values below.  They are set to
 * match TeamCode's RobotConstants; if you retune the robot's speeds or size there, change
 * them here too so MeepMeep keeps matching the robot.
 */
public final class RouteConstants
{
    private RouteConstants() {}

    public static double MAX_VEL       = 80;
    public static double MAX_ACCEL     = 40;
    public static double MAX_ANG_VEL   = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

    public static double SLW_VEL       = 60;
    public static double SLW_ACCEL     = 15;
    public static double SLW_ANG_VEL   = Math.toRadians(90);
    public static double SLW_ANG_ACCEL = Math.toRadians(90);

    public static double DT_TRACK_WIDTH = 16.5;
    public static double BOT_LEN = 18.0;
    public static double BOT_WID = 18.0;

    /** Name of the chassis (RobotConstants.Chassis), for poses that differ between bots. */
    public static String chassis = "B7252";

    /** ENABLE = a new piece at every addLocation, so the opmode can pause between them. */
    public static Field.AutonDebug autonDebug = null;

    public static final boolean logVerbose = true;

    /** Copy the driving limits and robot size in (called from TeamCode's RobotConstants.init). */
    public static void set(String chassisName,
                           double maxVel, double maxAccel, double maxAngVel, double maxAngAccel,
                           double slwVel, double slwAccel, double slwAngVel, double slwAngAccel,
                           double trackWidth, double botLen, double botWid)
    {
        chassis = chassisName;
        MAX_VEL = maxVel;
        MAX_ACCEL = maxAccel;
        MAX_ANG_VEL = maxAngVel;
        MAX_ANG_ACCEL = maxAngAccel;
        SLW_VEL = slwVel;
        SLW_ACCEL = slwAccel;
        SLW_ANG_VEL = slwAngVel;
        SLW_ANG_ACCEL = slwAngAccel;
        DT_TRACK_WIDTH = trackWidth;
        BOT_LEN = botLen;
        BOT_WID = botWid;
    }
}
