package org.firstinspires.ftc.teamcode.field;

/**
 * Field sizes and the choices for the auton menus (start, park, motif, ...).
 * Shared by TeamCode and MeepMeepTesting.
 */
@SuppressWarnings("unused")
public abstract class Field
{
    public enum Alliance {BLUE, RED}

    //  X axis parallel to red  alliance wall point toward    blue alliance
    //  Y axis parallel to blue alliance wall point away from red  alliance

    //The descriptions say the field is 12'x12', but our
    //practice field is actually slightly smaller at 141"
    private static final float X_WIDTH = 141.0f;
    private static final float Y_WIDTH = 141.0f;
    /* package */ static final float N_WALL_Y = Y_WIDTH/2.0f;
    /* package */ static final float E_WALL_X = X_WIDTH/2.0f;
    /* package */ static final float S_WALL_Y = -N_WALL_Y;
    /* package */ static final float W_WALL_X = -E_WALL_X;

    public static final float tileWidth        = 23.5f;
    public static final float oneTile          = tileWidth;
    public static final float halfTile         = tileWidth/2.0f;
    public static final float oneAndHalfTile   = 3.0f*tileWidth/2.0f;
    public static final float halfField        = X_WIDTH/2.0f;
    public static final float quadField        = X_WIDTH/4.0f;
    public static final float mmTargetHeight   = 6.0f;

	    public enum ParkPos implements PositionOption
    {
        CENTER_PARK,
        LEFT_PARK,
        RIGHT_PARK
    }

    public enum Route implements PositionOption
    {
        DROP_AND_PARK,
        REPEAT_4PT_JUNCTION

    }

    public enum StartPos implements PositionOption
    {
        START_SAMPLES,
        START_SPECIMENS
    }

    public enum AutonDebug implements PositionOption
    {
        ENABLE,
        DISABLE
    }

    public enum Start_Pos implements PositionOption{
        START_FAR_RED_SIMPLE,
        START_FARFAR_RED_SIMPLE,
        MAINFARROUTERED,
        FARMAINLYNEARRED,
        FARSTAYFARRED,
        START_WALL_RED4,
        START_GOAL_ROUTE_RED,
        START_FAR_RED6,
        MAINFARROUTEBLUE,
        FARMAINLYNEARBLUE,
        STARTFARSTAYFARBLUE,
        START_WALL_BLUE4,
        START_GOAL_ROUTE_BLUE,
        START_HUMANONLY,
        START_FAR_FAR_HUMAN_BLUE,
        START_FAR_BLUE6,
        START_FAR_BLUE_SIMPLE,
        START_FARFAR_BLUE_SIMPLE,


        }

    public enum Parks implements PositionOption
    {
        WALL,
        Park2,
        Park1
    }

    public enum Motif implements PositionOption
    {
        SHOOTGPP,
        SHOOTPGP,
        SHOOTPPG,
    }
    public enum Num_shots implements PositionOption
    {
        THREE,
        FOUR,
    }
    public enum Wiffle_Pos implements PositionOption
    {
        HUMAN1,
        PARK2,
        GATE3,
        GOAL4,
    }
    public enum Park_Pos implements PositionOption
    {
        NOPARKRED,
        INSIDEPRIMARYPARK1RED,
        INSIDEPRIMARYPARK2RED,
        OUTSIDEPRIMARYPARK1RED,
        OUTSIDEPRIMARYPARK2RED,
        INSIDEPRIMARYPARK1BLUE,
        INSIDEPRIMARYPARK2BLUE,
        OUTSIDEPRIMARYPARK1BLUE,
        OUTSIDEPRIMARYPARK2BLUE,
        N0PARKBLUE,
        DEFAULT
    }

}
