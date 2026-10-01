package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.MecanumBot;
import org.firstinspires.ftc.teamcode.robot.MecanumDriveLRR;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.BasicBot;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.firstinspires.ftc.teamcode.util.Point2d;

import java.util.Locale;

@Config
@TeleOp(name = "DtTeleop")
@Disabled
public class DtTeleop extends InitLinearOpMode
{
    private static final String TAG = "SJH_DTL";

    private void initPreStart()
    {
        BasicBot.OpModeType prevOpModeType = BasicBot.curOpModeType;
        BasicBot.curOpModeType = BasicBot.OpModeType.TELE;

        /* Initialize the hardware variables. */
        RobotLog.dd(TAG, "Initialize robot");

        RobotLog.dd(TAG, "Prev opmode type=%s.", prevOpModeType);
        /* Always initialize the sensors like the IMU and color sensors,
         *  even if you are Auto transitioning from another OpMode
        */
        robot.init(this, chas, true);

        mechDrv = (MecanumDriveLRR)(robot.drive);

        robot.setBcm(LynxModule.BulkCachingMode.MANUAL);

        Point2d autonEndPos = robot.getAutonEndPos();
        double autonEndHdg = robot.getAutonEndHdg();
        Pose2d startPose = new Pose2d(autonEndPos.getX(), autonEndPos.getY(), autonEndHdg);
        robot.drive.setPoseEstimate(startPose);
        RobotLog.dd(TAG, "Start Aend fHdg %.2f", Math.toDegrees(autonEndHdg));
        RobotLog.dd(TAG, "Start Pos %s", autonEndPos.toString());

        for(String k : robot.motors.keySet())
        {
            RobotLog.dd(TAG, "Motor mode in teleop = " + k + ":" +
                robot.motors.get(k).getMode());
        }

        RobotLog.dd(TAG, "Setting mode to RUN_USING_ENCODER for teleop");
        mechDrv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        transRcl =
//            new RateChangeLimiter(RobotConstants.MAX_ACCEL/RobotConstants.DT_MAX_IPS);
//        double maxDltAng =
//            RobotConstants.DT_TRACK_WIDTH * RobotConstants.MAX_ANG_ACCEL / (2*Math.PI);
//        rotRcl =
//            new RateChangeLimiter(maxDltAng/RobotConstants.DT_MAX_IPS);
    }

    private void update()
    {
        robot.update();

        l = 0;

        cnts=robot.getCnts();
        vels=robot.getVels();

        poseEstimate = robot.drive.getPoseEstimate();
    }

    private static final boolean VERBOSE = false;
    private void printTelem()
    {
        String cntStr = String.format(Locale.US,"CNTS: %d %d %d %d",
                cnts[0], cnts[1], cnts[2], cnts[3]);
        String velStr = String.format(Locale.US,"VELS: %d %d %d %d",
                (int)vels[0], (int)vels[1], (int)vels[2], (int)vels[3]);
        String posStr = String.format(Locale.US, "X:%.2f Y:%.2f H:%.2f",
            poseEstimate.getX(), poseEstimate.getY(), Math.toDegrees(poseEstimate.getHeading()));

        dashboard.displayText(l++, "Dir " + robot.getDriveDir());
        dashboard.displayText(l++, posStr);
        dashboard.displayText(l++, cntStr);
        dashboard.displayText(l++, velStr);
        dashboard.displayText(l++, String.format(Locale.US,"V:%.1f SP:%s", strtV, vcmpPID));
        dashboard.displayText(l++, String.format(Locale.US,"L_IN %4.2f L %4.2f", raw_lr, lr));
        dashboard.displayText(l++, String.format(Locale.US,"R_IN %4.2f R %4.2f", raw_fb, fb));
        dashboard.displayText(l++, String.format(Locale.US,"T_IN %4.2f T %4.2f", raw_turn, turn));
        if(VERBOSE) RobotLog.dd(TAG, "DRV:%.1f", drvTime);
        if(VERBOSE) RobotLog.dd(TAG, "TEL U:%.1f C:%.1f D:%.1f P:%.1f L:%.1f F:%.1f W:%.1f",
            u, c, d, p, L, f, w);
    }

    private void doLogging()
    {
        mechDrv.draw();
    }

    private void controlDrive()
    {
        if (robot.motors.size() == 0) return;

        raw_lr =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);
        raw_fb = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
        raw_turn =  gpad1.value(ManagedGamepad.AnalogInput.L_STICK_X);

        boolean rgt  = gpad1.pressed(ManagedGamepad.Button.D_RIGHT);
        boolean lft  = gpad1.pressed(ManagedGamepad.Button.D_LEFT);
        boolean fwd  = gpad1.pressed(ManagedGamepad.Button.D_UP);
        boolean bak  = gpad1.pressed(ManagedGamepad.Button.D_DOWN);
        boolean incr = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
        boolean decr = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
        boolean hspd = gpad1.pressed(ManagedGamepad.Button.R_TRIGGER);
        boolean dtrn = gpad1.pressed(ManagedGamepad.Button.L_TRIGGER);
        boolean tglF = gpad1.just_pressed(ManagedGamepad.Button.Y);

        if (tglF) useField = !useField;

        if(mechDrv.isBusy())
        {
            return;
        }

        if(lft) robot.motors.get("FL").setPower(0.5);
        if(bak) robot.motors.get("BL").setPower(0.5);
        if(rgt) robot.motors.get("BR").setPower(0.5);
        if(fwd) robot.motors.get("FR").setPower(0.5);

        lr = ishaper.shape(raw_lr, 0.02);
        fb = ishaper.shape(raw_fb, 0.02);
        turn = ishaper.shape(raw_turn, 0.02);

        if      (incr) dSpd += dStp;
        else if (decr) dSpd -= dStp;
        dSpd = Range.clip(dSpd, 0.0, 1.0);

        if (lft || rgt || fwd || bak)
        {
            if((lft || rgt) && dtrn)
            {
                turn = lft ? -dSpd : dSpd;
            }
            else
            {
                lr = lft ? -dSpd : rgt ? dSpd : 0.0;
                fb = bak ? -dSpd : fwd ? dSpd : 0.0;
                if ((lft || rgt) && (fwd || bak))
                {
                    lr /= spdScl;
                    fb /= spdScl;
                }
            }
        }

        Vector2d driveInput;
        if(useField)
        {
            // Rotate input vector by the inverse of current bot heading
            driveInput = new Vector2d(lr, fb).rotated(-poseEstimate.getHeading());
        }
        else
        {
            driveInput = new Vector2d(fb, -lr);
        }

        double maxCPS = RobotConstants.DT_SAF_CPS;
        if(hspd) maxCPS = RobotConstants.DT_MAX_CPS;
        double spdScl = maxCPS/RobotConstants.DT_MAX_CPS;

        driveInput = driveInput.times(spdScl);
        turn = turn * spdScl;

//        Pose2d velPose =
//            new Pose2d(transRcl.calculate(driveInput.norm()), -rotRcl.calculate(turn));
        Pose2d velPose = new Pose2d(driveInput, -turn);

        mechDrv.setWeightedDrivePower(velPose);
    }

    double drvTime;
    double u, c, d, p, L, f, w;
    private final ElapsedTime oTimer = new ElapsedTime();
    private final ElapsedTime opTimer = new ElapsedTime();
    private void processControllerInputs()
    {
        gpad2.update();
        opTimer.reset();
    }

    private void processDriverInputs()
    {
        gpad1.update();
        controlDrive();
        drvTime = opTimer.milliseconds();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this);

        initPreStart();

        dashboard.displayText(0, robot.getName() + " is ready");

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted() && !isStopRequested())
        {
            update();
            printTelem();
            doLogging();
            robot.finishFrame();
            robot.waitForTick(20);
        }

        strtV = robot.getBatteryVoltage();
        pidf = RobotConstants.SH_PID;
        vcmpPID = new PIDFCoefficients(pidf.p, pidf.i, pidf.d,
            pidf.f * 12.0/strtV);
        RobotLog.dd(TAG, "SHTPID: %s", vcmpPID);


        RobotLog.dd(TAG, "Mecanum_Driver starting");

        opTimer.reset();
        while (opModeIsActive())
        {
            oTimer.reset();
            update();
            u=opTimer.milliseconds();
            oTimer.reset();
            processControllerInputs();
            c=opTimer.milliseconds();
            oTimer.reset();
            processDriverInputs();
            d=opTimer.milliseconds();
            oTimer.reset();
            printTelem();
            p=opTimer.milliseconds();
            oTimer.reset();
            doLogging();
            L=opTimer.milliseconds();
            oTimer.reset();
            robot.finishFrame();
            f=opTimer.milliseconds();
            oTimer.reset();
            robot.waitForTick(20);
            w=opTimer.milliseconds();
            oTimer.reset();
        }
    }


    double dSpd = 0.0;
    double dStp = 0.1;

    static final double spdScl = Math.sqrt(2.0);
    Input_Shaper ishaper = new Input_Shaper();

    private boolean useField = false;
    private final MecanumBot robot = new MecanumBot();
    private MecanumDriveLRR  mechDrv;

    double raw_lr;
    double raw_fb;
    double raw_turn;
    double lr;
    double fb;
    double turn;

    Pose2d poseEstimate;

    int[] cnts = {0,0,0,0};
    double[] vels = {0,0,0,0};

    public static PIDFCoefficients pidf = RobotConstants.SH_PID;
    private PIDFCoefficients vcmpPID;

    private int l = 0;

    private double strtV;

//    private RateChangeLimiter transRcl;
//    private RateChangeLimiter rotRcl;
}