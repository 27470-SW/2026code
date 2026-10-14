package com.qualcomm.robotcore.util;

import java.util.Locale;

/**
 * MeepMeep's stand-in for the FTC SDK's RobotLog: prints to the console instead of logcat.
 *
 * Only MeepMeep uses this file; the robot uses the real RobotLog.  Do NOT add methods that the
 * real RobotLog doesn't have (the shared code would compile, then crash on the robot).
 */
public class RobotLog
{
    public static void vv(String tag, String format, Object... args)                      { log("V", tag, null, format, args); }
    public static void vv(String tag, String message)                                     { log("V", tag, null, message); }
    public static void vv(String tag, Throwable t, String format, Object... args)         { log("V", tag, t, format, args); }
    public static void vv(String tag, Throwable t, String message)                        { log("V", tag, t, message); }

    public static void dd(String tag, String format, Object... args)                      { log("D", tag, null, format, args); }
    public static void dd(String tag, String message)                                     { log("D", tag, null, message); }
    public static void dd(String tag, Throwable t, String format, Object... args)         { log("D", tag, t, format, args); }
    public static void dd(String tag, Throwable t, String message)                        { log("D", tag, t, message); }

    public static void ii(String tag, String format, Object... args)                      { log("I", tag, null, format, args); }
    public static void ii(String tag, String message)                                     { log("I", tag, null, message); }
    public static void ii(String tag, Throwable t, String format, Object... args)         { log("I", tag, t, format, args); }
    public static void ii(String tag, Throwable t, String message)                        { log("I", tag, t, message); }

    public static void ww(String tag, String format, Object... args)                      { log("W", tag, null, format, args); }
    public static void ww(String tag, String message)                                     { log("W", tag, null, message); }
    public static void ww(String tag, Throwable t, String format, Object... args)         { log("W", tag, t, format, args); }
    public static void ww(String tag, Throwable t, String message)                        { log("W", tag, t, message); }

    public static void ee(String tag, String format, Object... args)                      { log("E", tag, null, format, args); }
    public static void ee(String tag, String message)                                     { log("E", tag, null, message); }
    public static void ee(String tag, Throwable t, String format, Object... args)         { log("E", tag, t, format, args); }
    public static void ee(String tag, Throwable t, String message)                        { log("E", tag, t, message); }

    private static void log(String level, String tag, Throwable t, String format, Object... args)
    {
        String msg = format;
        if (args.length > 0)
        {
            try { msg = String.format(Locale.US, format, args); }
            catch (Exception e) { msg = format; }
        }
        System.out.println(level + "/" + tag + ": " + msg);
        if (null != t) t.printStackTrace(System.out);
    }
}
