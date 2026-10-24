package org.firstinspires.ftc.teamcode.field;

import com.qualcomm.robotcore.util.RobotLog;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;

import static org.firstinspires.ftc.teamcode.robot.RobotConstants.*;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.TrajEnum.*;

/* ==============================================================================================
 * Route: the route builder.  This file should NOT need to change from season to season.
 *
 * It is shared by TeamCode (the robot) and MeepMeepTesting, so it must stay plain Java:
 * no Android, no FTC SDK, no robot hardware.
 *
 * The pieces of a season's auton are layered on top of it:
 *
 *   Route          (SharedCode)       addLocation, addFunction, addEvent, ifBlock, whileBlock...
 *     Decode_Route (SharedCode)       poses, reusable pieces, the functions routes may call, picks the route
 *       RouteRunner  (TeamCode)         what each function does on the robot; what Decode_Auton runs
 *       RouteRunner  (MeepMeepTesting)  what each function does in MeepMeep (prints it)
 *
 * The route files (MAINFARROUTERED, ...) are in SharedCode and are used by both.
 * ============================================================================================== */
@SuppressWarnings("unused")
public abstract class Route
{
     private static final String TAG = "SJH_RTE";
    protected static final boolean VERBOSE = RobotConstants.logVerbose;

     protected static TrajectoryVelocityConstraint defVelLim =
       RobotConstants.defVelConstraint;
     protected static TrajectoryAccelerationConstraint defAccelLim =
       RobotConstants.defAccelConstraint;

     protected static TrajectoryVelocityConstraint slwVelLim =
       RobotConstants.slwVelConstraint;
     protected static TrajectoryAccelerationConstraint slwAccelLim =
       RobotConstants.slwAccelConstraint;

     protected static final boolean strafeDropX = false;

     public Route(PositionOption startPos)
     {
         this.startPos = startPos;

         totalDur = 0;

         /* All points of interests will be based off of quadrant I - positive x, positive y */
         /* Reflection over the x-axis will be done for the alliance side */
         /* Reflection over the y-axis will be done on which side you line up */
         /* Blue Left = Quad I, Blue Right = Quad II, Red Right =  */
         /* For headings: Counter clockwise is positive, Clockwise is negative */
         if (alliance == Field.Alliance.BLUE)
		 {
             /* y scalar value  */
             /* Blue side is in quadrant I & II of the 2D coordinate system */
             sy = 1;
             /* Bot should point south or towards the negative Y direction */
             /* point the bot in the clockwise direction */
             sh = -1;
             sf = 0;
             sx = -1;
             flip = Math.toRadians(180);

                 sr = 0;

         }
		 else
		 {
             /* Red side is in quadrant III & IV of the 2D coordinate system */
             sy = 1;
             /* Bot should point north or in the positive Y direction on red side */
             /* point the bot in the counter clockwise direction */
             sh = 1;
             sf = -1;
             sx =1;
             flip = Math.toRadians(0);


                 /* Red Left quadrant III */
                 sr = 0;


         }
     }

     /** Builds the route.  Call once, at the end of the constructor of the season's route
      *  class (e.g. Decode_Route), after all its settings and poses are stored. */
     protected void buildRoute()
     {
         if (INIT_TRAJ_2 != RobotConstants.trajType)
         {
             initTrajectories();
         }
         if (INIT_TRAJ_1 != RobotConstants.trajType)
         {
             initTrajectories2();
         }
     }

     /** Override: build the route with addLocation, addFunction, ... then finalizeTrajSeq(). */
     protected void initTrajectories2()
     {
         finalizeTrajSeq();
     }

     protected void initTrajectories()
     {

     }

     public String toString()
     {
         return "";
     }

     protected PositionOption startPos;
     protected PositionOption direction;
     protected PositionOption under_hive;


    protected Field.Alliance alliance;

     /** ENABLE = a new piece at every addLocation, so the opmode can pause between them.
      *  Decode_Auton sets this from its menu choice before building routes. */
     public static Field.AutonDebug autonDebug;

     /** The park choice; addParkPosition routes for this key get added at the end. */
     protected PositionOption parkChoice;

     /* Alliance mirroring (set in the constructor) */
     protected int sx;
     protected int sy;
     protected double sh;
     protected int sf;
     protected int sr;
     protected double flip;

     final static int MAX_SEGMENTS = 32;

	 /* Map for new route, leaves old route in tact in case we need to revert back to it to validate */
     public EnumMap<SegmentState, TrajectorySequence>  segmentStateMap = new EnumMap<>(Route.SegmentState.class);
     TrajectorySequence fullSeq;

     /* New way to define map */
     public List<TrajectorySequence> trajList;
     public Map<PositionOption,TrajectorySequence> parkMap = new HashMap<>();
     private Pose2d lastPose;
     private Pose2d firstPose;
     private TrajectorySequenceBuilder traj;
     protected double totalDur;

    public Pose2d getEnd(){
        if(null!= trajList && trajList.size() != 0){
            return trajList.get(trajList.size()-1).end();
        }
        else { return null; }
    }

     public enum Movement
     {
          START, //used for first position, i.e. no moving to this location
          LINE,
          STRAFE,
          STRAFE_LEFT,
          STRAFE_RIGHT,
          SPLINE,
          FORWARD,
          BACK,
          TURN
     }
     public enum Heading
     {
          HEAD_DEFAULT,
          HEAD_CONSTANT,
          HEAD_LINEAR,
          HEAD_SPLINE
     }

     public enum Action
     {
          WAIT,
          TANGENT,
          SLOW
     }

     public enum TeamElement
     {
         LEFT,
         RIGHT,
         CENTER
     }

     public Pose2d getLastPose(){
        return lastPose;
     }

   public double calcTimeOrSomething(){
       double cumTime = 0.0;

       if(null != trajList)
       {


           int seqNum = 0;



           for (TrajectorySequence tSeq : trajList)
           {
               String seqName = String.valueOf(seqNum++);
               for (int i = 0; i < tSeq.size(); i++)
               {
                   SequenceSegment seg = tSeq.get(i);

                   if (seg instanceof TrajectorySegment)
                   {
                       Trajectory t = ((TrajectorySegment) seg).getTrajectory();
                       cumTime += t.duration();
                   }
                   else
                   {
                       if (seg instanceof TurnSegment)
                       {
                           TurnSegment ts = (TurnSegment) seg;
                           cumTime += ts.getDuration();
                       }
                       else if (seg instanceof WaitSegment)
                       {
                           WaitSegment ws = (WaitSegment) seg;
                           cumTime += ws.getDuration();
                       }
                   }
               }
           }

       }
       return cumTime;
    }



    public TrajectorySequence getTrajSeq(int trajNum){
         TrajectorySequence nextTraj = null;
         switch (trajNum){
             case 0:
                 nextTraj = fullSeq;
         }

         return nextTraj;
    }

     boolean justStarted = true;

     public void addLocation(Pose2d loc, Movement move, Heading head)
     {
         addLocation(loc, move, head, false, 0);
     }

    public void addLocation(Pose2d loc, Movement move, Heading head, double customHeadValue)
    {
        addLocation(loc, move, head, true, customHeadValue);
    }

    private boolean newTraj = false;


    public void makeNewTraj()
    {
        if (null == lastPose) return;   // nothing to split before the START location
        if (null != split) return;      // each copy becomes its own piece when they rejoin
        closePiece();
        openPiece(lastPose);
    }

    /* ==========================================================================================
     * CONDITIONALS: waitUntil, ifBlock / elseIfBlock / elseBlock / endIf, whileBlock / endWhile
     *
     * The route is still built ONCE, at init, as pieces (trajList).  Alongside the pieces we
     * build "program": a list of steps that the opmode walks through during the match.  This is
     * the state machine: each step is a state, and each step says which step comes next.
     *
     *   RUN    drive one piece, then go to the next step
     *   CHECK  hold still and check the condition until it is true or timeoutSec passes
     *          (timeoutSec 0 = check once):  true -> next step,  false/timeout -> step "target"
     *   JUMP   go to step "target" (end of an if branch, bottom of a while loop)
     *
     * A route without any conditionals is just RUN piece 0, RUN piece 1, ... exactly like before.
     *
     * A piece can only start where the robot actually is.  So:
     *   - a whileBlock body must end where the loop started (checked at init), and
     *   - if the parts of an ifBlock end in different places, the route "splits": Route keeps one
     *     copy per end spot.  Functions and WAITs after endIf go into every copy, and so does the
     *     next addLocation; after that every copy is at the same spot and they rejoin.
     * ========================================================================================== */

    /** A yes/no question the route can ask during the match, e.g. route::motifFound */
    public interface Condition
    {
        boolean check();
    }

    public enum StepType { RUN, CHECK, JUMP }

    public static class Step
    {
        public final StepType type;
        public final int piece;             // RUN: index into trajList
        public final Condition condition;   // CHECK
        public final double timeoutSec;     // CHECK: how long to wait for true (0 = check once)
        public int target = -1;             // CHECK (when false/timeout), JUMP: step to go to
        public final String label;          // for the log

        Step(StepType type, int piece, Condition condition, double timeoutSec, String label)
        {
            this.type = type;
            this.piece = piece;
            this.condition = condition;
            this.timeoutSec = timeoutSec;
            this.label = label;
        }
    }

    /** The state machine the opmode runs (see Decode_Auton.doAuton). */
    public final List<Step> program = new ArrayList<>();

    private enum BlockType { IF, WHILE }

    private static class Block
    {
        final BlockType type;
        final Pose2d startPose;
        final int topStep;                       // whileBlock: the condition step to loop back to
        int pendingCheck = -1;                   // CHECK step whose false target is not known yet
        final List<Arrival> arrivals = new ArrayList<>();    // every way out of this ifBlock
        boolean hasElse = false;
        String currentBranch;

        Block(BlockType type, Pose2d startPose, int topStep, String currentBranch)
        {
            this.type = type;
            this.startPose = startPose;
            this.topStep = topStep;
            this.currentBranch = currentBranch;
        }
    }

    private final Deque<Block> blocks = new ArrayDeque<>();

    /* One way of arriving somewhere: where the robot is, and which steps lead here (their
     * "target" gets filled in once we know where "here" is in the program).  While the route is
     * split, each copy is an Arrival with its own builder. */
    private static class Arrival
    {
        Pose2d pose;
        TrajectorySequenceBuilder builder;                  // split copies only
        final List<Integer> entrySteps = new ArrayList<>(); // steps whose target must lead here
        boolean fallsThrough = false;                       // also reached by just going to the next step

        Arrival(Pose2d pose)
        {
            this.pose = pose;
        }
    }

    /* Not null after an endIf whose parts end in different places: one copy per end spot,
     * until the next addLocation brings them all to the same spot. */
    private List<Arrival> split = null;

    /** Hold still (robot is stopped here) until the condition is true or timeoutSec passes. */
    public void waitUntil(Condition condition, double timeoutSec)
    {
        startBlockBoundary("waitUntil");
        int i = addStep(new Step(StepType.CHECK, -1, condition, timeoutSec,
                String.format(Locale.US, "waitUntil (max %.1f s)", timeoutSec)));
        program.get(i).target = i + 1;      // true or timeout: either way, carry on
        openPiece(lastPose);
    }

    /** Check the condition once (robot is stopped here); run this block only if it is true. */
    public void ifBlock(Condition condition)
    {
        ifBlock(condition, 0);
    }

    /** Hold still up to timeoutSec waiting for the condition; run this block if it became true. */
    public void ifBlock(Condition condition, double timeoutSec)
    {
        startBlockBoundary("ifBlock");
        Block b = new Block(BlockType.IF, lastPose, program.size(), "ifBlock");
        b.pendingCheck = addStep(new Step(StepType.CHECK, -1, condition, timeoutSec, checkLabel("ifBlock", timeoutSec)));
        blocks.push(b);
        openPiece(b.startPose);
    }

    public void elseIfBlock(Condition condition)
    {
        elseIfBlock(condition, 0);
    }

    public void elseIfBlock(Condition condition, double timeoutSec)
    {
        Block b = currentBlock(BlockType.IF, "elseIfBlock");
        if (b.hasElse)
            throw new IllegalStateException("elseIfBlock can't come after elseBlock.");
        endBranch(b, true);
        program.get(b.pendingCheck).target = program.size();
        b.pendingCheck = addStep(new Step(StepType.CHECK, -1, condition, timeoutSec, checkLabel("elseIfBlock", timeoutSec)));
        b.currentBranch = "elseIfBlock";
        openPiece(b.startPose);
    }

    public void elseBlock()
    {
        Block b = currentBlock(BlockType.IF, "elseBlock");
        if (b.hasElse)
            throw new IllegalStateException("An ifBlock can only have one elseBlock.");
        endBranch(b, true);
        program.get(b.pendingCheck).target = program.size();
        b.pendingCheck = -1;
        b.hasElse = true;
        b.currentBranch = "elseBlock";
        openPiece(b.startPose);
    }

    public void endIf()
    {
        Block b = currentBlock(BlockType.IF, "endIf");
        endBranch(b, false);
        blocks.pop();

        if (!b.hasElse)
        {
            // Without an elseBlock, "condition false" skips the block: the robot stays where it was.
            Arrival skip = new Arrival(b.startPose);
            skip.entrySteps.add(b.pendingCheck);
            b.arrivals.add(skip);
        }

        // Group the ways out of this ifBlock by where the robot ends up.
        List<Arrival> spots = new ArrayList<>();
        for (Arrival a : b.arrivals)
        {
            Arrival same = null;
            for (Arrival spot : spots)
            {
                if (posesMatch(spot.pose, a.pose)) same = spot;
            }
            if (null == same)
            {
                same = new Arrival(a.pose);
                spots.add(same);
            }
            same.entrySteps.addAll(a.entrySteps);
            same.fallsThrough |= a.fallsThrough;
        }

        if (spots.size() == 1)
        {
            // Every way ends at the same spot: continue together from here.
            int next = program.size();
            for (int step : spots.get(0).entrySteps) program.get(step).target = next;
            openPiece(spots.get(0).pose);
        }
        else
        {
            // Different spots: keep one copy of the route per spot until the next addLocation.
            for (Arrival spot : spots) spot.builder = newBuilder(spot.pose);
            split = spots;
            traj = null;
        }
    }

    /** Repeat the block while the condition is true. Checked before every lap, robot stopped. */
    public void whileBlock(Condition condition)
    {
        startBlockBoundary("whileBlock");
        Block b = new Block(BlockType.WHILE, lastPose, program.size(), "whileBlock");
        b.pendingCheck = addStep(new Step(StepType.CHECK, -1, condition, 0, "whileBlock"));
        blocks.push(b);
        openPiece(b.startPose);
    }

    public void endWhile()
    {
        Block b = currentBlock(BlockType.WHILE, "endWhile");
        checkNoPending("endWhile");
        checkNotSplit("endWhile");
        closePiece();
        if (!posesMatch(lastPose, b.startPose))
            throw new IllegalStateException("The whileBlock starts at " + fmt(b.startPose)
                    + " but its last location is " + fmt(lastPose)
                    + ". The robot must end each lap where the loop started, so the next lap can begin.");
        blocks.pop();

        Step back = new Step(StepType.JUMP, -1, null, 0, "endWhile (back to whileBlock)");
        back.target = b.topStep;
        addStep(back);
        program.get(b.pendingCheck).target = program.size();
        openPiece(b.startPose);
    }

    private static String checkLabel(String what, double timeoutSec)
    {
        return timeoutSec > 0 ? String.format(Locale.US, "%s (wait up to %.1f s)", what, timeoutSec) : what;
    }

    private int addStep(Step s)
    {
        program.add(s);
        return program.size() - 1;
    }

    /** Every finished piece goes through here: into trajList, plus a RUN step in the program. */
    private void addPiece(TrajectorySequence seq)
    {
        if (null == trajList) trajList = new ArrayList<TrajectorySequence>();
        trajList.add(seq);
        int idx = trajList.size() - 1;
        addStep(new Step(StepType.RUN, idx, null, 0, "drive piece " + idx));
    }

    /** Finish the current piece (if it has anything in it). Never throws on an empty piece. */
    private void closePiece()
    {
        if (null != traj && !traj.isEmpty())
        {
            addPiece(traj.build());
        }
        traj = null;
    }

    private void openPiece(Pose2d from)
    {
        lastPose = from;
        traj = newBuilder(from);
    }

    private TrajectorySequenceBuilder newBuilder(Pose2d from)
    {
        return new TrajectorySequenceBuilder(
                from,
                defVelLim, defAccelLim,
                RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);
    }

    private void checkNotSplit(String what)
    {
        if (null != split)
            throw new IllegalStateException("The ifBlock just before " + what
                    + " ends in different places, so the robot could be in more than one spot here."
                    + " Add an addLocation (where every path should go next) before " + what + ".");
    }

    /** Run a builder change once, or once per copy while the route is split. */
    private void inEachCopy(Runnable change)
    {
        if (null == split)
        {
            change.run();
            return;
        }
        for (Arrival copy : split)
        {
            traj = copy.builder;
            lastPose = copy.pose;
            change.run();
            copy.builder = traj;
        }
        traj = null;
    }

    /* Turn each split copy into steps: its own piece (if it has anything in it), then a JUMP whose
     * target is filled in later.  The last copy may instead just continue to the next step.
     * The copy that is reached by falling through must come first, because falling through
     * lands on whatever step comes next.  Returns the ways out (where each copy ends). */
    private List<Arrival> flush(List<Arrival> copies, boolean lastMayFallThrough)
    {
        List<Arrival> order = new ArrayList<>();
        for (Arrival copy : copies)
        {
            if (copy.fallsThrough) order.add(0, copy);
            else order.add(copy);
        }

        List<Arrival> exits = new ArrayList<>();
        for (int i = 0; i < order.size(); i++)
        {
            Arrival copy = order.get(i);
            int first = program.size();
            for (int step : copy.entrySteps) program.get(step).target = first;
            traj = copy.builder;
            closePiece();

            Arrival exit = new Arrival(copy.pose);
            if (lastMayFallThrough && i == order.size() - 1)
                exit.fallsThrough = true;
            else
                exit.entrySteps.add(addStep(new Step(StepType.JUMP, -1, null, 0, "rejoin")));
            exits.add(exit);
        }
        return exits;
    }

    /* addLocation while split: add the move to every copy, then all copies are at loc. */
    private void rejoinAt(Pose2d loc, Movement move, Heading head, boolean custom, double customHeadValue)
    {
        if (move == Movement.START)
            throw new IllegalStateException("START can't come right after an endIf.");

        List<Arrival> copies = split;
        split = null;   // so addLocation below works on one copy at a time
        Double slow = pendingSlow;
        Double tangent = pendingTangent;
        for (Arrival copy : copies)
        {
            traj = copy.builder;
            lastPose = copy.pose;
            pendingSlow = slow;
            pendingTangent = tangent;
            newTraj = false;   // never split a copy in the middle
            addLocation(loc, move, head, custom, customHeadValue);
            copy.builder = traj;
            copy.pose = loc;
        }
        newTraj = false;

        List<Arrival> exits = flush(copies, true);
        int next = program.size();
        for (Arrival exit : exits)
        {
            for (int step : exit.entrySteps) program.get(step).target = next;
        }
        openPiece(loc);
    }

    private void checkNoPending(String what)
    {
        if (null != pendingSlow || null != pendingTangent)
            throw new IllegalStateException("A SLOW or TANGENT right before " + what
                    + " has no location to apply to. Put it right before the addLocation it is for.");
    }

    private void startBlockBoundary(String what)
    {
        checkNotSplit(what);
        if (null == lastPose)
            throw new IllegalStateException(what + " must come after the START location.");
        checkNoPending(what);
        closePiece();
    }

    private Block currentBlock(BlockType type, String what)
    {
        if (blocks.isEmpty() || blocks.peek().type != type)
            throw new IllegalStateException(what + " without a matching "
                    + (type == BlockType.IF ? "ifBlock" : "whileBlock") + ".");
        return blocks.peek();
    }

    /** Close the current part of an ifBlock and remember how it gets out (and where it ends). */
    private void endBranch(Block b, boolean jumpToEnd)
    {
        checkNoPending("the end of " + b.currentBranch);
        if (null != split)
        {
            // This part ended with its own ifBlock that ended in different places:
            // each copy becomes a way out of this part.
            List<Arrival> copies = split;
            split = null;
            b.arrivals.addAll(flush(copies, !jumpToEnd));
        }
        else
        {
            closePiece();
            Arrival a = new Arrival(lastPose);
            if (jumpToEnd)
                a.entrySteps.add(addStep(new Step(StepType.JUMP, -1, null, 0, "end of " + b.currentBranch)));
            else
                a.fallsThrough = true;
            b.arrivals.add(a);
        }
        traj = null;
    }

    private static boolean posesMatch(Pose2d a, Pose2d b)
    {
        return Math.abs(a.getX() - b.getX()) < 0.5
                && Math.abs(a.getY() - b.getY()) < 0.5
                && Math.abs(Angle.normDelta(a.getHeading() - b.getHeading())) < Math.toRadians(1.0);
    }

    private static String fmt(Pose2d p)
    {
        return String.format(Locale.US, "(x %.1f, y %.1f, heading %.0f deg)",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }

    /** Log the state machine, one step per line. */
    public void printProgram()
    {
        for (int i = 0; i < program.size(); i++)
        {
            Step s = program.get(i);
            switch (s.type)
            {
                case RUN:
                    TrajectorySequence seq = trajList.get(s.piece);
                    RobotLog.dd(TAG, "step %d: %s  %s -> %s", i, s.label, fmt(seq.start()), fmt(seq.end()));
                    break;
                case CHECK:
                    RobotLog.dd(TAG, "step %d: %s: if false go to step %d", i, s.label, s.target);
                    break;
                case JUMP:
                    RobotLog.dd(TAG, "step %d: %s: go to step %d", i, s.label, s.target);
                    break;
            }
        }
    }

    /* ==========================================================================================
     * DRY RUN: walk the program the same way the opmode does, but with no robot.
     * MeepMeep uses this to show the path the robot would really take, with each condition
     * answered by MeepMeep's RouteRunner.  A check that comes back false and has a timeout holds still
     * for the whole timeout, like the robot would.
     * ========================================================================================== */
    public interface DryRunListener
    {
        void drive(TrajectorySequence piece);
        void holdStill(double seconds);
    }

    /** Stops a dry run whose whileBlock never ends. */
    public static final int DRY_RUN_MAX_STEPS = 1000;

    public void dryRun(DryRunListener out)
    {
        int step = 0;
        int count = 0;
        while (step >= 0 && step < program.size())
        {
            if (++count > DRY_RUN_MAX_STEPS)
            {
                RobotLog.ee(TAG, "dry run stopped after %d steps (does a whileBlock never end?)", DRY_RUN_MAX_STEPS);
                return;
            }
            Step s = program.get(step);
            switch (s.type)
            {
                case RUN:
                    out.drive(trajList.get(s.piece));
                    step++;
                    break;

                case CHECK:
                    boolean result;
                    try
                    {
                        result = s.condition.check();
                    }
                    catch (Exception e)
                    {
                        RobotLog.ee(TAG, "Condition failed (%s); treating it as false", e.toString());
                        result = false;
                    }
                    RobotLog.dd(TAG, "step %d %s: %s", step, s.label,
                            result ? "true" : (s.timeoutSec > 0 ? "timed out" : "false"));
                    if (!result && s.timeoutSec > 0) out.holdStill(s.timeoutSec);
                    step = result ? step + 1 : s.target;
                    break;

                case JUMP:
                    step = s.target;
                    break;
            }
        }
    }

    public void addParkPosition(PositionOption parkEnum, Pose2d loc, Movement move, Heading head)
    {
        Pose2d origLastPose = lastPose;
        addLocation(loc, move, head, false, 0);
        parkMap.put(parkEnum, traj.build());
        try
  	    {
            RobotLog.dd(TAG, "putting park position, %d", parkEnum);
        }
  	    catch (Exception e)
  	    {
              RobotLog.dd(TAG, "failure putting park position in log");
        }
        traj = null;
        lastPose = origLastPose;
    }

    private boolean samePose(Pose2d loc){
        boolean sp = false;
        if(loc.getY() == lastPose.getY() && loc.getX() == lastPose.getX()){
            sp = true;
            if(loc.getHeading() != lastPose.getHeading()){
                //TODO: throw compiler warning
            }
        }
        return sp;
    }

    public void addLocation(Pose2d loc, Movement move, Heading head, boolean custom, double customHeadValue)
    {
        if (null != split)
        {
            rejoinAt(loc, move, head, custom, customHeadValue);
            return;
        }
        RobotLog.dd(TAG, "adding Location: x:%f, y:%f, h:%f, move:%s,head:%s", loc.getX(), loc.getY(), Math.toDegrees(loc.getHeading()), move, head);
        if(null != lastPose)
  	    {
              if(!samePose(loc)) {
             if(justStarted)
  	         {
                 justStarted = false;
                 if (null == trajList) trajList = new ArrayList<TrajectorySequence>();
             }
             if(newTraj)
  	         {
	           makeNewTraj();
               newTraj = false;
	         }
             else if(autonDebug == Field.AutonDebug.ENABLE)
	         {
	             //when in debug mode, default to a new Traj every time unless something set it to false
	             newTraj = true;
	         }          
             else
	         {
	             newTraj = false;
		         //when not in debug, default to false unless something set it to true;
	         }

              if (move != Movement.START)
              {
                  if (null != pendingSlow)
                  {
                      if (pendingSlow <= 0) {
                          traj.setConstraints(slwVelLim, slwAccelLim);
                          traj.setTurnConstraint(RobotConstants.SLW_ANG_VEL, RobotConstants.SLW_ANG_ACCEL);
                      }
                      else {
                          traj.setConstraints(
                                  new MinVelocityConstraint(Arrays.asList(
                                          new AngularVelocityConstraint(pendingSlow * MAX_ANG_VEL),
                                          new MecanumVelocityConstraint(pendingSlow * MAX_VEL, DT_TRACK_WIDTH))),
                                  new ProfileAccelerationConstraint(pendingSlow * MAX_ACCEL));
                          traj.setTurnConstraint(pendingSlow * MAX_ANG_VEL, pendingSlow * RobotConstants.MAX_ANG_ACCEL);
                      }
                      pendingSlow = null;
                  }
                  else{
                      traj.resetConstraints();
                      traj.resetTurnConstraint();
                  }
                  if (pendingTangent != null) {
                      traj.setTangent(pendingTangent);
                      pendingTangent = null;
                  }
                  else {    //resets default tangent value, so it only ever applies to one location
                      traj.clearTangent();
                  }
              }

             switch(move)
  	         {
                  case START:
                      //acts as a restart; should not normally be used
                      firstPose = loc;
                      justStarted = true;
                          traj = new TrajectorySequenceBuilder(
                                  firstPose,
                                  defVelLim, defAccelLim,
                                  RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);
                      break;
                  case LINE:
                      switch (head)
                      {
                         case HEAD_LINEAR:
                           traj.lineToLinearHeading(loc);
                           break;
                         case HEAD_CONSTANT:
                           traj.lineToConstantHeading(loc.vec());
                           break;
                         case HEAD_SPLINE:
                           traj.lineToSplineHeading(loc);
                           break;
                         case HEAD_DEFAULT:
                         default:
                           traj.lineTo(loc.vec());
                           break;
                      }
                      break;
                  case STRAFE:
                      traj.strafeTo(loc.vec());
                      break;
                  default: //Spline or anything else
                      double headValue;

                      if(custom)
                      {
                          headValue = customHeadValue;
                      }
                      else
                      {
                          headValue = loc.getHeading();
                      }
                      switch (head)
                      {
                            case HEAD_LINEAR:
                              traj.splineToLinearHeading(loc,headValue);
                              break;
                            case HEAD_SPLINE:
                              traj.splineToSplineHeading(loc,headValue + flip);
                              break;
                            case HEAD_CONSTANT:
                              traj.splineToConstantHeading(loc.vec(),headValue + flip);
                              break;
                            case HEAD_DEFAULT:
                            default:
                              traj.splineTo(loc.vec(),headValue);
                              break;
                      }
                      break;
                  }
             }
        }
        else
  	    {
  	        //What should happen when first called with START
            firstPose = loc;
            justStarted = true;
            traj = new TrajectorySequenceBuilder(
                    firstPose,
                    defVelLim, defAccelLim,
                    RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);
        }

        lastPose = loc; //Setup for next time

     }

     public void addMovement(Movement move, Double amount)
     {
          checkNotSplit("FORWARD/BACK/STRAFE/TURN (use an addLocation instead)");
          switch (move)
  	      {
               case STRAFE_RIGHT:
                 traj.strafeRight(amount);
                 break;
               case STRAFE_LEFT:
                 traj.strafeLeft(amount);
                 break;
               case FORWARD:
                 traj.forward(amount);
                 break;
               case BACK:
                 traj.back(amount);
                 break;
               case TURN:
                 traj.turn(amount);
                 break;
          }
     }

     public void addFunction(MarkerCallback callback)
     {
          inEachCopy(() -> {
               anchorFunctions();
               traj.addTemporalMarker(callback);
          });
     }

     public void addFunction(MarkerCallback callback, double time)
     {
          inEachCopy(() -> {
               anchorFunctions();
               traj.addTemporalMarker(time, callback);
          });
     }

     /* A function needs something in its piece to attach to.  If the piece is still empty
      * (e.g. right after START, waitUntil, ifBlock or elseBlock), add a zero-length wait so
      * the function always runs, even if no move follows it. */
     private void anchorFunctions()
     {
          if (traj.isEmpty()) traj.waitSeconds(0);
     }

     private Double pendingTangent = null;
     private Double pendingSlow = null;

     public void addEvent(Action act, double value)
     {
          switch(act)
  	      {
              case WAIT:
                  inEachCopy(() -> traj.waitSeconds(value));
                  break;
              case TANGENT:
          /*        try{          //cath error when trying to add tangent to first trajectory. TODO: make this not needed
				      makeNewTraj();
			      }catch(Exception e){

                  }
                  newTraj = false;
                  traj.setTangent(value);
           */
                  RobotLog.dd(TAG, "Setting tangent as %f", value);
                  pendingTangent = value;
                  break;
              case SLOW:
        /*          try{  //Need to make a new traj in order to make this one slow
                      makeNewTraj();
                  }catch (Exception e){}    //error if that traj didn't have anything in it yet
                  RobotLog.dd(TAG, "Setting traj as SLOW");
                  if(value <= 0) {
                      traj = new TrajectorySequenceBuilder(
                              lastPose,
                              slwVelLim, slwAccelLim,
                              RobotConstants.SLW_ANG_VEL, RobotConstants.SLW_ANG_ACCEL);
                  }
                  else
                  {
                      TrajectoryVelocityConstraint vc =
                              new MinVelocityConstraint(Arrays.asList(
                                      new AngularVelocityConstraint(value/MAX_VEL*MAX_ANG_VEL),
                                      new MecanumVelocityConstraint(value, DT_TRACK_WIDTH)
                              ));
                      TrajectoryAccelerationConstraint ac = new ProfileAccelerationConstraint(value/MAX_VEL*MAX_ACCEL);
                      traj = new TrajectorySequenceBuilder(
                              lastPose,
                              vc, ac,
                              value, value/MAX_VEL*RobotConstants.MAX_ANG_ACCEL);
                  }
         */
                  pendingSlow = value;
                  break;

          }
     }

    public void addTrajectory(TrajectorySequence trajectory)
    {
         if(null == trajList)
  	     {
             addLocation(trajectory.start(), Movement.START, Heading.HEAD_DEFAULT);
         }
         if(lastPose != trajectory.start())
  	     {
             addLocation(trajectory.start(), Movement.SPLINE, Heading.HEAD_SPLINE);
         }
         closePiece();
         addPiece(trajectory);
         lastPose = trajectory.end();
         newTraj = true;
    }

    public void addParkTrajectory(PositionOption parkEnum, TrajectorySequence trajectory)
    {
         parkMap.put(parkEnum, trajectory);
    }

    public void printLastPose(){
        try{
            TrajectorySequence temp = trajList.get(trajList.size()-1);
            if(VERBOSE) {  RobotLog.dd(TAG, "Trajectory Sequence from %s to %s, dur=%f",
                    temp.start(), temp.end(), temp.duration());}
        }catch(Exception e){
            if(VERBOSE) { RobotLog.dd(TAG, "Unable to printLastPose");}
        }
    }

    public void newRoute(){

    }

    public void finalizeTrajSeq()
    {
        RobotLog.dd(TAG, "finalizing a TrajSeq");
        if (!blocks.isEmpty())
        {
            throw new IllegalStateException("The route ends inside a "
                    + (blocks.peek().type == BlockType.IF ? "ifBlock: add endIf()" : "whileBlock: add endWhile()") + ".");
        }
        if (null != split)
        {
            // The route ends while split (e.g. each part parks somewhere different): finish each copy.
            List<Arrival> copies = split;
            split = null;
            List<Arrival> exits = flush(copies, true);
            int end = program.size();
            for (Arrival exit : exits)
            {
                for (int step : exit.entrySteps) program.get(step).target = end;
            }
        }
        closePiece();   // the last piece (skipped if empty)
        /* Finalize Traj Seq */
        if(null != trajList)
	    {
            RobotLog.dd(TAG,"trajList does not = null");
             if(null != fullSeq)
	         {
                 addTrajectory(fullSeq);
             }

             TrajectorySequenceBuilder full = new TrajectorySequenceBuilder(
                     firstPose,
                     defVelLim, defAccelLim,
                     RobotConstants.MAX_ANG_VEL, RobotConstants.MAX_ANG_ACCEL);

             double cumTime = 0.0;
            if(VERBOSE) { RobotLog.dd(TAG, "Done Building trajectories");}
             int seqNum = 0;
             TrajectorySequence parkTraj = parkMap.get(parkChoice);
             if(null != parkTraj) {
                 addPiece(parkTraj);
             }

             for (TrajectorySequence tSeq : trajList)
  	         {
                    String seqName = String.valueOf(seqNum++);
                    //TrajectorySequence tSeq = e.getValue();
                 if(VERBOSE) { RobotLog.dd(TAG, "Trajectory Sequence %s len=%d dur=%.2f",
                         seqName, tSeq.size(), tSeq.duration());}

                   for (int i = 0; i < tSeq.size(); i++)
  	          	   {
                         SequenceSegment seg = tSeq.get(i);
                       if(VERBOSE) {  RobotLog.dd(TAG, "Seg %d %s to %s in %.2f",
                           i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());}

                         if (seg instanceof TrajectorySegment)
                         {
                               Trajectory t = ((TrajectorySegment) seg).getTrajectory();
                               full.addTrajectory(t);
                             if(VERBOSE) { RobotLog.dd(TAG, "TrajSeg %d %s to %s in %.2f",
                                 i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());}

                               for (TrajectoryMarker m : ((TrajectorySegment) seg).getTrajectory().getMarkers())
                               {
                                   if(VERBOSE) { RobotLog.dd(TAG, "  marker: time=%.2f", m.getTime());}
                               }
                               cumTime += t.duration();
                         }
  	          		    else
  	          	        {
                             if (seg instanceof TurnSegment)
  	          		         {
                                   TurnSegment ts = (TurnSegment) seg;
                                   full.turn(ts.getTotalRotation());
                                 if(VERBOSE) { RobotLog.dd(TAG, "TurnSeg %d %s to %s in %.2f",
                                     i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());}

                                   for (TrajectoryMarker m : seg.getMarkers())
  	          		               {
                                       full.addTemporalMarker(m.getTime(), m.getCallback());
                                   }
                                   cumTime += ts.getDuration();
                             }
  	          		         else if (seg instanceof WaitSegment)
  	          		         {
                                   WaitSegment ws = (WaitSegment) seg;
                                   full.waitSeconds(ws.getDuration());
                                 if(VERBOSE) {  RobotLog.dd(TAG, "WaitSeg %d %s to %s in %.2f",
                                     i, seg.getStartPose(), seg.getEndPose(), seg.getDuration());}

                                   for (TrajectoryMarker m : seg.getMarkers())
  	          		               {
                                       full.addTemporalMarker(m.getTime(), m.getCallback());
                                       if(VERBOSE) { RobotLog.dd(TAG, "  marker: time=%.2f", m.getTime());}
                                   }
                                   cumTime += ws.getDuration();
                              }
                         }
                    }
             }
         fullSeq = full.build();
         printProgram();
         }
    }

    public Pose2d getStart(){
        return firstPose;
    }

    public enum SegmentState
    {
        MOVE_FROM_START,
		/* LEFT SIDE Trajectory Segments */
        DROP_CONE_3PT_JUNCTION,
        MOVE_FROM_3PT_JUNCTION,
        CONE_GRAB,
		/* RIGHT SIDE Trajectory Segments */
		MOVE_FROM_SHARED_5PT_JUNCTION,
		CONE_GRAB_FROM_SHARED_5PT_JUNCTION,
		/* END Of Specific Segments */		
        DROP_CONE_4PT_JUNCTION,
        REVERSE_FROM_4PT_JUNCTION,
        DROP_CONE_4PT_JUNCTION_REPEAT,
        REVERSE_FROM_4PT_JUNCTION_REPEAT,
        DROP_CONE_OPP_5PT_JUNCTION,
        PARK_RIGHT,
        PARK_LEFT,
        PARK_CENTER
    }
}
