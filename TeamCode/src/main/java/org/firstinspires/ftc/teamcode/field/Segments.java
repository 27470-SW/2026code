package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.BasicBot;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Segment;

import java.util.Vector;


@SuppressWarnings("unused")
public abstract class Segments
{
    private static final String TAG = "SJH_SEG";

    Segments(Field.Alliance alliance)
    {
        RobotLog.dd(TAG, "In base Segments ctor.  Alliance=%s", alliance);

        Vector<Point2d> pts = initPoints();
        Vector<Point2d> points;
        if(alliance == Field.Alliance.RED)
        {
            RobotLog.dd(TAG, "InitRedpoints");
            points = initRedPoints(pts);
        }
        else
        {
            RobotLog.dd(TAG, "InitBluepoints");
            points = initBluePoints(pts);
        }

        segments  = initSegments(points);
    }

    protected Vector<Point2d> initPoints()
    {
        return new Vector<>(MAX_SEGMENTS);
    }

    void addPoint(Vector<Point2d> points,
                  BasicBot.DriveDir dir,
                  double speed,
                  @SuppressWarnings("SameParameterValue") double tune,
                  Segment.TargetType targetType,
                  Segment.Action action,
                  Point2d pt)
    {
        segDirs.add(dir);
        segSpeeds.add(speed);
        ttypes.add(targetType);
        actions.add(action);
        tuners.add(tune);
        points.add(pt);
    }

    public final Segment[] getSegments()
    {
        return segments;
    }

    private Vector<Point2d> initRedPoints(Vector<Point2d> inpts)
    {
        Vector<Point2d> rpts = new Vector<>(inpts.size());

        rpts.addAll(inpts);
        return rpts;
    }

    private Vector<Point2d> initBluePoints(Vector<Point2d> inpts)
    {
        Vector<Point2d> bpts = new Vector<>(inpts.size());

        for(Point2d rpt : inpts)
        {
            bpts.add(convertRtoB(rpt));
        }
        return bpts;
    }

    private Segment[] initSegments(Vector<Point2d> pts)
    {
        int numSegs = pts.size() - 1;
        Segment[] pathSegs = new Segment[numSegs];
        Segment seg;
        for(int s = 0; s < numSegs; ++s)
        {
            String sname = pts.get(s+1).getName();

            seg = new Segment(sname, pts.get(s), pts.get(s+1));
            seg.setDir(segDirs.get(s));
            seg.setSpeed(segSpeeds.get(s));
            seg.setAction(actions.get(s));
            seg.setTgtType(ttypes.get(s));
            seg.setDrvTuner(tuners.get(s));

            RobotLog.ii(TAG, "setting up segment %s %s %s %4.1f tune: %4.2f",
                    seg.getName(), seg.getStrtPt(), seg.getTgtPt(),
                    seg.getFieldHeading(), seg.getDrvTuner());

            pathSegs[s] = seg;
        }
        return pathSegs;
    }

    @SuppressWarnings("unused")
    private Segment getSegment(String name, Segment[] segs)
    {
        for (Segment pathSeg : segs)
        {
            String n = pathSeg.getName();
            if (n.equals(name)) return pathSeg;
        }
        return null;
    }

    public Point2d convertRtoB(Point2d rpt)
    {
        double bx =  rpt.getX();
        double by = -rpt.getY();

        String nm = "B" + rpt.getName().substring(1);

        RobotLog.dd(TAG, "convRtoB %s to %s %4.1f %4.1f", rpt.getName(), nm,
                bx, by);

        return new Point2d(nm, bx, by);
    }

    public String toString()
    {
        StringBuilder sbldr = new StringBuilder();
        sbldr.append("Segments\n");
        for (Segment segment : segments)
        {
            sbldr.append(segment.toString()).append("\n");
        }
        return sbldr.toString();
    }

    final static int    MAX_SEGMENTS = 16;

    private final Segment[] segments;

    private final Vector<Segment.Action>     actions   = new Vector<>(MAX_SEGMENTS);
    private final Vector<Double>             segSpeeds = new Vector<>(MAX_SEGMENTS);
    private final Vector<BasicBot.DriveDir> segDirs   = new Vector<>(MAX_SEGMENTS);
    private final Vector<Double>             tuners    = new Vector<>(MAX_SEGMENTS);
    private final Vector<Segment.TargetType> ttypes    = new Vector<>(MAX_SEGMENTS);
}