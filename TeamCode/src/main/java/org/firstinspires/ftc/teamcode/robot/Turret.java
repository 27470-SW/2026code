package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.RobotConstants.CLOSE_BLUE_HIVE_X;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.CLOSE_BLUE_HIVE_Y;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.CLOSE_RED_HIVE_X;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.CLOSE_RED_HIVE_Y;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.FAR_BLUE_HIVE_X;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.FAR_BLUE_HIVE_Y;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.FAR_RED_HIVE_X;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.FAR_RED_HIVE_Y;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.INVERT_DIRECTION;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SERVO_CENTER_TRIM_1;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SERVO_CENTER_TRIM_2;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.SERVO_RANGE_RAD;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.TURRET_OFFSET_X;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.TURRET_OFFSET_Y;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.PreferenceMgr;

import kotlin.math.UMathKt;


public class Turret {

    public Turret(HardwareMap map, String servoT1Name, String servoT2Name, double servoT1_0, double servoT2_0, MecanumDriveLRR mech)
    {
        mechDrv = mech;
        servo1Name = servoT1Name;
        servo2Name = servoT2Name;
        this.hwMap = map;
        try {
            servoT1 = hwMap.get(Servo.class, servoT1Name);
            servoT2 = hwMap.get(Servo.class, servoT2Name);
        }catch(Exception e){
            RobotLog.ee(TAG, "Unable to find turret %s %s", servoT1Name, servoT2Name);
        }
        //percent = 0.5;
      //  onIntake = 1;
      //  offIntake = 0.5;
        servoT1Zero = servoT1_0;
        servoT2Zero = servoT2_0;
        alliance = Field.Alliance.valueOf(PreferenceMgr.getAllianceColor());
        RobotLog.dd(TAG, "initialized Turret");

    }
    public void init (double pos) {
        setPosition(pos);
    }
    public void setManPosition(double pos){
        autoAline = false;
        setPosition(pos);
    }
    public void setPosition(double pos){
        percent = pos;

        if(servoT1 == null) return;
        servoT1.setPosition(getRealPos(pos, servoT1Zero));
        RobotLog.dd(TAG, "%s Percent:%f POS:%f", servo1Name, percent, pos);
        if(servoT2 == null) return;
        servoT2.setPosition(getRealPos(pos, servoT2Zero));
        RobotLog.dd(TAG, "%s Percent:%f POS:%f", servo2Name, percent, pos);

    }
    public void moveTurretAmount(double amount){
        percent = percent + amount;
        setPosition(percent);
        RobotLog.dd(TAG, "moving to position: %f", percent);
        autoAline = false;
    }

    public void setAutoAline(boolean auto) {
        autoAline = auto;
    }


        public double getRealPos(double pos, double zero){

        return zero + pos + 0.5;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public static double angleToServoCommand(double turretAngle) {
        double cmd = turretAngle * GEAR_RATIO / SERVO_RANGE_RAD;
        if (INVERT_DIRECTION) cmd = -cmd;
        return clamp(cmd, -0.5, 0.5);
    }


    public void update(){
        if (autoAline) {
            Pose2d robotLocation = mechDrv.getRealPoseEstimate();
            // setPosition(percent);
            //setIntakePower(percent);
            // 1) Turret pivot on the field = robot position + offset rotated by heading
            double cos = Math.cos(robotLocation.getHeading());
            double sin = Math.sin(robotLocation.getHeading());
            double pivotX;
            double pivotY;
            double servoCommand;
            double servo1Pos;
            double servo2Pos;
            pivotX = robotLocation.getX() + TURRET_OFFSET_X * cos - TURRET_OFFSET_Y * sin;
            pivotY = robotLocation.getY() + TURRET_OFFSET_X * sin + TURRET_OFFSET_Y * cos;
            getHiveLocation(robotLocation);
            double dx = hiveX - pivotX;
            double dy = hiveY - pivotY;
            double pivotDist = Math.hypot(dx, dy);
            double bearing = Math.atan2(dy, dx);
            double desired = bearing - robotLocation.getHeading(); // - TURRET_ZERO_ANGLE;//Look out for needing to get it from pie to -pie
            servoCommand = angleToServoCommand(desired);
            servo1Pos = clamp(0.5 + servoCommand + SERVO_CENTER_TRIM_1, 0.0, 1.0);
            servo2Pos = clamp(0.5 + servoCommand + SERVO_CENTER_TRIM_2, 0.0, 1.0);
            servoT1.setPosition(servo1Pos);
            servoT2.setPosition(servo2Pos);


        logCount++;
        if(logCount == 10) {
            RobotLog.dd(TAG, "%s Percent:%f PIVOTY:%f PIVOTX:%f ServoPos:%f Pos1:%f Pos2::%f HIVEX:%f HIVEY:%f Bearing:%f, Desired:%f ", servo1Name, percent, pivotY, pivotX, servoCommand, servo1Pos, servo2Pos, hiveX, hiveY, Math.toDegrees(bearing), desired);
            logCount = 0;
        }
        }
        else {
            RobotLog.dd(TAG, "%s Percent:%f autoAline:%b servoT1:%f servoT2:%f", servo2Name, percent, autoAline, servoT1.getPosition(), servoT2.getPosition());
        }
    }
    int logCount = 0;

    public void getHiveLocation(Pose2d robotLocation){
        double robotY = robotLocation.getY ();
        if (alliance == Field.Alliance.RED){
            if (robotY > 0) {
                hiveX = FAR_RED_HIVE_X;
                hiveY = FAR_RED_HIVE_Y;
            }
        else {//Setting hive to close one.
            hiveX = CLOSE_RED_HIVE_X;
            hiveY = CLOSE_RED_HIVE_Y;
            }
        }
        else {
            //alliance == Field.Alliance.BLUE
        if (robotY > 0) {
                hiveX = FAR_BLUE_HIVE_X;
                hiveY = FAR_BLUE_HIVE_Y;
            }
        else {
            hiveX = CLOSE_BLUE_HIVE_X;
            hiveY = CLOSE_BLUE_HIVE_Y;
            }
        }
    }


    public double percent;
    private static final String TAG = "turretclass";
    public Servo servoT2;
    public Servo servoT1;
    public  double servoT2Zero;
    public double servoT1Zero;
    protected HardwareMap hwMap;
    public double onIntake;
    public double offIntake;
    public boolean Triggr;
    public int K1R2;
    public String servo1Name;
    public String servo2Name;
    private MecanumDriveLRR  mechDrv;
    private double hiveX;
    private double hiveY;
    public Field.Alliance alliance;
    public boolean autoAline = true;

}
