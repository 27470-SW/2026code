package org.firstinspires.ftc.teamcode.field;

import java.util.Locale;

/**
 * Logging for the shared route code.  The shared code can't use RobotLog (that is Android),
 * so it logs through here instead:
 *   - on the robot, RobotRoute points this at RobotLog (shows up in logcat like before)
 *   - in MeepMeep it prints to the console
 */
public final class RouteLog
{
    public interface Sink
    {
        void debug(String tag, String msg);
        void error(String tag, String msg);
    }

    private static Sink sink = new Sink()
    {
        @Override public void debug(String tag, String msg) { System.out.println(tag + ": " + msg); }
        @Override public void error(String tag, String msg) { System.err.println(tag + ": " + msg); }
    };

    private RouteLog() {}

    public static void setSink(Sink newSink)
    {
        sink = newSink;
    }

    public static void dd(String tag, String format, Object... args)
    {
        sink.debug(tag, format(format, args));
    }

    public static void ee(String tag, String format, Object... args)
    {
        sink.error(tag, format(format, args));
    }

    private static String format(String format, Object... args)
    {
        if (args.length == 0) return format;
        try
        {
            return String.format(Locale.US, format, args);
        }
        catch (Exception e)
        {
            return format;
        }
    }
}
