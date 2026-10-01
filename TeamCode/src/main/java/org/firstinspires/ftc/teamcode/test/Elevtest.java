package org.firstinspires.ftc.teamcode.test;

import android.util.SparseArray;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.MotorComponent;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

import java.util.Locale;

/**
 * This OpMode steps n servos positions up and down based on D-Pad user inputs.
 * This code assumes a Servo configured with the name "testservo#".
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 */
@TeleOp(name = "Elevatortest", group = "Test")
//@Disabled
public class Elevtest extends InitLinearOpMode
{

    private static final double INCREMENT = 0.02;     // servo spd incr
    private static final int     CYCLE_MS = 50;       // period of each cycle
    private static final double   MAX_PWR =  1.0;     // Maximum servo pwr
    private static final double   MIN_PWR =  0.0;
    private static final double   IPS = 0.5;
    // Minimum servo pwr
    private double pwr = 0.5;

    PwmControl.PwmRange extRange = new PwmControl.PwmRange(500, 2500);

    // Define class members
    private static final int MAX_SERVOS = 1;

    private static final String TAG = "SJH_SST";

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this);
        MotorComponent elevServo = new MotorComponent("testelev", hardwareMap);
        elevServo.init(IPS);
//        SparseArray<CRServo> servos = new SparseArray<>(MAX_SERVOS);

        DigitalChannel maxstop = null;
        DigitalChannel minstop = null;

        int minTouchCnt = 0;
        int maxTouchCnt = 0;

        for(int m = 0; m < MAX_SERVOS; m++)
        {
            String servoName = "testservo" + m;

         }

        // Wait for the start button
        dashboard.displayText(0, "Press Start to run elevator test.");
        while(!isStarted())
        {

            //CRServo srv = servos.get(m);
            if(elevServo != null)
                    dashboard.displayText(1,
                        String.format(Locale.US, "SRV_%d %.2f", 1, elevServo.getPwr()));

            sleep(10);
        }
        waitForStart();

        while(opModeIsActive()) {
            gpad1.update();
            elevServo.update();
            boolean fwd = gpad1.pressed(ManagedGamepad.Button.D_UP);
            boolean bck = gpad1.pressed(ManagedGamepad.Button.D_DOWN);
            boolean newfwd = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean newback = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean zeroize = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean fullize = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean incrSpd = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean decrSpd = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
            boolean extend = gpad1.just_pressed(ManagedGamepad.Button.Y);
            boolean chngDir = gpad1.just_pressed(ManagedGamepad.Button.X);
            boolean enable = gpad1.just_pressed(ManagedGamepad.Button.A);
            boolean disable = gpad1.just_pressed(ManagedGamepad.Button.B);

            if (incrSpd) pwr += INCREMENT;
            pwr = Math.min(MAX_PWR, pwr);
            if (decrSpd) pwr -= INCREMENT;
            pwr = Math.max(MIN_PWR, pwr);
            if (zeroize) pwr = 0.5;
            if (fullize) pwr = MAX_PWR;
            // Display the current value
            dashboard.displayText(MAX_SERVOS,
                    String.format(Locale.US, "Servo pos %4.2f %s %s", pwr, newfwd, fwd));
            //for(int m = 0; m < MAX_SERVOS; m++)
            //{
            //CRServo srv = servos.get(m);
            if (elevServo == null) continue;
            CRServoImplEx srvEx = null;
//            if (srv instanceof CRServoImplEx) {
//                srvEx = (CRServoImplEx) srv;
//            }

            if (minstop != null && !minstop.getState()) {
                elevServo.moveAtRate(MIN_PWR);
                minTouchCnt++;
                if (minTouchCnt == 0) {
                    RobotLog.dd(TAG, "Min stop hit");
                }

                continue;
            } else {
                minTouchCnt = 0;
            }

            if (maxstop != null && !maxstop.getState()) {
                elevServo.moveAtRate(MIN_PWR);
                maxTouchCnt++;
                if (maxTouchCnt == 0) {
                    RobotLog.dd(TAG, "Max stop hit");
                }

                continue;
            } else {
                maxTouchCnt = 0;
            }

            if (disable && srvEx != null) {
                srvEx.setPwmDisable();
                continue;
            }
            if (enable && srvEx != null) {
                srvEx.setPwmEnable();
                continue;
            }
            if (extend && srvEx != null) {
                srvEx.setPwmRange(extRange);
            }

            if (chngDir)
            {
                DcMotorSimple.Direction curdir = elevServo.getCurDir();
                DcMotorSimple.Direction newdir = DcMotorSimple.Direction.FORWARD;
                if (curdir != newdir)
                {
                    elevServo.setDir(newdir);
                }

            }

            if (fwd)
            {
                elevServo.moveAtRate(pwr);
                if (newfwd) {
                    RobotLog.dd(TAG, "Fwd just pressed.  Pwr=%4.2f", pwr);
                }
            }
            else if (bck)
            {
                elevServo.moveAtRate(-pwr);

                if (newback) {
                    RobotLog.dd(TAG, "Bak just pressed.  Pwr=%4.2f", -pwr);
                }
            }
            else
            {
                elevServo.moveAtRate(0.5);
            }
            //}

            dashboard.displayText(MAX_SERVOS + 1, "Press Stop to end test.");
            dashboard.displayText(MAX_SERVOS + 2, "Incr pwr : R_BUMP");
            dashboard.displayText(MAX_SERVOS + 3, "Decr pwr : L_BUMP");
            dashboard.displayText(MAX_SERVOS + 4, "Zero pwr : Dpad left");
            dashboard.displayText(MAX_SERVOS + 4, "Max  pwr : Dpad right");
            dashboard.displayText(MAX_SERVOS + 5, "Change Dir: : X");
            dashboard.displayText(MAX_SERVOS + 5, "Extend Range: : Y");
            dashboard.displayText(MAX_SERVOS + 5, "Disable PWM: : A");
            dashboard.displayText(MAX_SERVOS + 5, "Enable PWM: : B");

            sleep(CYCLE_MS);
        }
    }
}
