package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.sequencesegment.SequenceSegment;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.sequencesegment.TrajectorySegment;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.sequencesegment.TurnSegment;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.sequencesegment.WaitSegment;

import java.util.ArrayList;
import java.util.List;

/* ==============================================================================================
 * RouteRunner (MeepMeepTesting): the Decode_Route that MeepMeepTesting builds and shows.
 * It says what each route function does IN MEEPMEEP (it just prints its name), answers the
 * route's conditions, and turns the route into the path MeepMeep animates (toMeepMeep).
 *
 * The routes and poses are in SharedCode: the same files the robot runs.
 * TeamCode has its own RouteRunner with the real robot version of these functions.
 * ============================================================================================== */
public class RouteRunner extends Decode_Route
{
    /* ----------------------------------------------------------------------------------------
     * Answers for the route's conditions.  Change these to see the other path in MeepMeep.
     * ---------------------------------------------------------------------------------------- */
    public static boolean MOTIF_FOUND = true;

    public RouteRunner(PositionOption startPos,
                         Field.Park_Pos parkPos,
                         Field.Wiffle_Pos lastLocation,
                         Field.Motif motif,
                         Field.Num_shots numshot)
    {
        super(startPos, parkPos, lastLocation, motif, numshot);   // builds the route
    }

    private static void say(String what)
    {
        System.out.println(what);
    }

    @Override protected void shootFar()                        { say("Turn on wheel (far)"); }
    @Override protected void shootWiffleClose()                { say("Shoot Balls Close"); }
    @Override protected void shoot1()                          { say("Shoot Transition 1"); }
    @Override protected void shoot2()                          { say("Shoot Transition 2"); }
    @Override protected void shoot3()                          { say("Shoot Transition 3"); }
    @Override protected void shootAllThree()                   { say("Shoot all three"); }
    @Override protected void shoot3Wiffles()                   { say("Shoot 3 wiffles (far)"); }
    @Override public    void shoot3Wifflesfixed()              { say("Shoot 3 wiffles"); }
    @Override protected void wheelOff()                        { say("Wheel off"); }
    @Override protected void intakeOn()                        { say("IntakeOn"); }
    @Override protected void intakeOff()                       { say("Intake Off"); }
    @Override protected void intakeonandthreeTransitionsDown() { say("Intake on, transitions down"); }
    @Override protected void transitonUp()                     { say("Transition Up ALL"); }
    @Override public    void deliverSample()                   { say("Deliver sample"); }
    @Override public    void armToDrop()                       { say("Arm to drop"); }

    @Override
    public boolean motifFound()
    {
        say("motifFound? " + MOTIF_FOUND);
        return MOTIF_FOUND;
    }

    /**
     * The route as one MeepMeep TrajectorySequence, following the path the robot would really
     * take: each ifBlock / whileBlock / waitUntil is answered by the conditions above.
     */
    public TrajectorySequence toMeepMeep()
    {
        final List<SequenceSegment> segments = new ArrayList<>();
        dryRun(new DryRunListener()
        {
            @Override
            public void drive(org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence piece)
            {
                for (int i = 0; i < piece.size(); i++)
                {
                    segments.add(convert(piece.get(i)));
                }
            }

            @Override
            public void holdStill(double seconds)
            {
                Pose2d here = segments.isEmpty() ? getStart() : segments.get(segments.size() - 1).getEndPose();
                segments.add(new WaitSegment(here, seconds, new ArrayList<>()));
            }
        });
        return new TrajectorySequence(segments);
    }

    /* The shared code's segments and MeepMeep's are the same thing in different packages. */
    private static SequenceSegment convert(org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment seg)
    {
        if (seg instanceof org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment)
        {
            return new TrajectorySegment(
                    ((org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment) seg).getTrajectory());
        }
        if (seg instanceof org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment)
        {
            org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment t =
                    (org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment) seg;
            return new TurnSegment(t.getStartPose(), t.getTotalRotation(), t.getMotionProfile(), t.getMarkers());
        }
        return new WaitSegment(seg.getStartPose(), seg.getDuration(), seg.getMarkers());
    }
}
