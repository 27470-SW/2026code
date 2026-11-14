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

    public enum Direction implements PositionOption
    {
        //values(direction):
        FROM_FIRST_TO_SECOND,
        FROM_SECOND_TO_FIRST

    }

    public enum Under_Hive implements PositionOption
    {
        //values(direction):
        UNDER_HIVE,
        OUTSIDE_OF_HIVE

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
        START_FIRST,
        START_SECOND,
        START_SECOND_PARK_CLOSE


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
        PARK1,
        PARK2,
        NOPARK,

    }

    public enum Partner_Start_Pos implements PositionOption
    {
        STARTFIRST,
        STARTSECOND,
    }
    public enum Partner_Path implements PositionOption
    {
        INSIDE,
        OUTSIDE,
        NEITHER,

    }
    public enum Partner_Flower implements PositionOption
    {
        FLOWER1,
        FLOWER2,
        BOTHFLOWERS,
        NEITHER,
    }
    public enum Partner_Garden implements PositionOption
    {
        TRUE,
        FALSE,
    }
    public enum Hive_Tipped implements PositionOption
    {
        TRUE,
        FALSE,

    }
}
