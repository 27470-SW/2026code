package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.field.Field.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.field.Field.Alliance.RED;
import static org.firstinspires.ftc.teamcode.robot.Motors.MotorModel.GOBILDA_5202_13_7;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BABYHUMMINGBIRDZERO1;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BABYHUMMINGBIRDZERO2;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BLUE_CELL_APRIL_TAG1;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BLUE_CELL_APRIL_TAG1_FIRST;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BLUE_CELL_APRIL_TAG2;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BLUE_CELL_APRIL_TAG2_FIRST;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BLUE_CELL_APRIL_TAG3;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BLUE_CELL_APRIL_TAG3_FIRST;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BLUE_CELL_APRIL_TAG4;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.BLUE_CELL_APRIL_TAG4_FIRST;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.HUMMINGBIRDSTARTINGPOS;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.HUMMINGBIRDZERO1;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.HUMMINGBIRDZERO2;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.RED_CELL_APRIL_TAG1;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.RED_CELL_APRIL_TAG1_FIRST;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.RED_CELL_APRIL_TAG2;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.RED_CELL_APRIL_TAG2_FIRST;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.RED_CELL_APRIL_TAG3;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.RED_CELL_APRIL_TAG3_FIRST;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.RED_CELL_APRIL_TAG4;
import static org.firstinspires.ftc.teamcode.robot.RobotConstants.RED_CELL_APRIL_TAG4_FIRST;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.PreferenceMgr;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import java.util.List;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class MecanumBot extends BasicBot {
    private static final String TAG = "SJH_MEC";

    //public Claw claw = null;
    //Still need this for now for backward compatibility


    //public ServoIntake servoIntake =null;
    public CrAzYintake crAzYIntake = null;
    public Shooter shooterN = null;
    public Shooter shooterP = null;
    public Hummingbird babyHummingbird = null;
    public Hummingbird hummingBird = null;
    public MotorComponent transitionWheels = null;

    public double logIntakeCurSpd = 0.0;

    public MecanumBot() {
        super();

        name = "27470";
        bulkCachingMode = LynxModule.BulkCachingMode.MANUAL;

        DRIVE_GEARS =
                new double[]{RobotConstants.DT_MOTOR.getGear(), RobotConstants.DT_EXT_GEAR_RATIO};

        WHEEL_DIAMETER_INCHES = RobotConstants.DT_WHEEL_DIAM;

        gyroInverted = false;
    }

    @Override
    public void init(LinearOpMode op, boolean initDirSensor) {
        RobotLog.dd(TAG, "MecanumBot init " + name);
        initCore(op);

        this.initDirSensor = initDirSensor;
        initSensors();
        initDriveMotors();
        initJointControl();
        initCapabilities();
    }

    @Override
    protected void initSensors() {
        super.initSensors();

        boolean clrGood = false;
        boolean dist1Good = false;

//        try
//        {
//            colorSensor = hwMap.get(NormalizedColorSensor.class, "color1");
//            colorSensor.setGain(25.0f);
//            clrGood = true;
//        }
//        catch(Exception e)
//        {
//            RobotLog.ee(TAG, "ERROR get colorSensor\n" + e.toString());
//        }
//        try
//        {
//            rearDistSensor = hwMap.get(DistanceSensor.class, "rearDist");
//            dist1Good = true;
//        }
//        catch(Exception e)
//        {
//            RobotLog.ee(TAG, "ERROR get distanceSensor1\n" + e.toString());
//        }
//
//        try
//        {
//            pixelColorSensor = hwMap.get(ColorSensor.class, "color2");
//            dist1Good = true;
//        }
//        catch(Exception e)
//        {
//            RobotLog.ee(TAG, "ERROR get distanceSensor1\n" + e.toString());
//        }
//
//        capMap.put("sensor", clrGood && super.imuGood && dist1Good);
    }


    @Override
    protected void initDriveMotors() {
        RobotLog.dd(TAG, TAG + "Initializing drive motors");
        drive = new MecanumDriveLRR(imu);
        //Roadrunner mech drive starts with FL motor and goes counterclockwise (FL,BL,BR,FR)
        try {
            List<DcMotorEx> mtrs = ((MecanumDriveLRR) drive).getMotors();
            motors.put("FL", mtrs.get(0));
            motors.put("BL", mtrs.get(1));
            motors.put("BR", mtrs.get(2));
            motors.put("FR", mtrs.get(3));

            capMap.put("drivetrain", true);
            CommonUtil.getInstance().getDashboard().displayText(6, "TRMB_MOTORS: " + mtrs.size());
        } catch (Exception e) {
            RobotLog.ee(TAG, "ERROR get motors in MecanumBot\n" + e.toString());
        }
    }

    private DigitalChannel redLED1;
    private DigitalChannel greenLED1;
    private DigitalChannel redLED2;
    private DigitalChannel greenLED2;


    public int pixelPieces = 0;

    @Override
    protected void initJointControl() {
        RobotLog.dd(TAG, "init Lifter MecanumBot\n");
        //claw = new Claw (hwMap);
        //claw.maxClosed = .85;
        //claw.maxOpen = .35;

        //elbowMotor = new MotorComponent("elbow", hwMap);
        //extenderMotor = new MotorComponent("extend", hwMap);
        //bucketServo = new MotorComponent("bucket", hwMap);
        //wristServo = new ServoComponent("wrist", hwMap);
        //droneLauncherServo = new ServoComponent("drone", hwMap);
        //sweeperServo1 = new MotorComponent("sweeper1", hwMap);
        //sweeperServo2 = new MotorComponent("sweeper2", hwMap);
        //intake = new Intake(hwMap);

        // servoIntake = new ServoIntake(hwMap);
        crAzYIntake = new CrAzYintake(hwMap);
        shooterP = new Shooter(hwMap);
        shooterN = new Shooter(hwMap);
        babyHummingbird = new Hummingbird(hwMap, "baby1", "baby2", BABYHUMMINGBIRDZERO1, BABYHUMMINGBIRDZERO2);
        hummingBird = new Hummingbird(hwMap, "bird1", "bird2", HUMMINGBIRDZERO1, HUMMINGBIRDZERO2);
        transitionWheels = new MotorComponent("shooter1", hwMap);


        /*redLED1 = hwMap.get(DigitalChannel.class, "red1");
        greenLED1 = hwMap.get(DigitalChannel.class, "green1");
        redLED2 = hwMap.get(DigitalChannel.class, "red2");
        greenLED2 = hwMap.get(DigitalChannel.class, "green2");
        redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
        redLED2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED2.setMode(DigitalChannel.Mode.OUTPUT);

        elbowMotor.init(RobotConstants.EL_EXT_MOT, -2);
        elbowMotor.setDir(RobotConstants.EL_DIR);
//        elbowMotor.setMode(STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setLevelOffset(
            RobotConstants.EL_LEVS[0],
            RobotConstants.EL_LEVS[1],
            RobotConstants.EL_LEVS[2],
            RobotConstants.EL_LEVS[3],
            RobotConstants.EL_LEVS[4],
            RobotConstants.EL_LEVS[5]);
        armLevel =0;

        extenderMotor.init(RobotConstants.EL_EX_MOT, 4.72);
        extenderMotor.setDir(RobotConstants.EXT_DIR);
        //extenderMotor.setMode(STOP_AND_RESET_ENCODER);
        extenderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extenderMotor.setLevelOffset(
            RobotConstants.EX_LEVS[0],
            RobotConstants.EX_LEVS[1]);

        sweeperServo1.init(RobotConstants.SWP_SRV);
        sweeperServo1.setDir(RobotConstants.SWP_DIR);

        sweeperServo2.init(RobotConstants.SWP_SRV);
        sweeperServo2.setDir(RobotConstants.SWP_DIR2);

        bucketServo.init(RobotConstants.SWP_SRV);
        bucketServo.setDir(RobotConstants.SWP_DIR2);
        bucketServo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        wristServo.init(0,.82);
        wristServo.moveAt(1);

        droneLauncherServo.init(0,.60);
        droneLauncherServo.moveAt(1.0);
        droneLauncherServo.moveTo(0.25);
        if (droneLauncherServo != null)
        {
            RobotLog.dd(TAG,"drone launcher was init");
        }
        intake.init();
        */


        shooterP.initShooter("shootP", RobotConstants.BallType.POLLEN);
        shooterN.initShooter("shootN", RobotConstants.BallType.NECTAR);
        transitionWheels.init(GOBILDA_5202_13_7, 1);
        transitionWheels.setZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);
        hummingBird.init(0.2);
        babyHummingbird.init(0);

        crAzYIntake.init();

        //  slides.setMode(STOP_AND_RESET_ENCODER);        //TODO: make not happen when comming back from auton;
        //elbowMotor.init(RobotConstants.EL_EXT_M, -2);

    }

    private final ElapsedTime updTimer = new ElapsedTime();


    public void update() {
        updTimer.reset();
        //RobotLog.dd(TAG, "pose(7) is now: %s", drive.getLocalizer().getPoseEstimate().toString());

        super.update();
        //RobotLog.dd(TAG, "pose(8) is now: %s", drive.getLocalizer().getPoseEstimate().toString());

        double botTime = updTimer.milliseconds();
        updTimer.reset();
        // RobotLog.dd(TAG, "pose(9) is now: %s", drive.getLocalizer().getPoseEstimate().toString());


        if (shooterP != null) {
            shooterP.update();
        }
        if (crAzYIntake != null) {
            crAzYIntake.update();
        }
        if (shooterN != null) {
            shooterN.update();
        }
        if (babyHummingbird != null) {
            babyHummingbird.update();
        }
        if (hummingBird != null) {
            hummingBird.update();
        }
        if (transitionWheels != null) {
            transitionWheels.update();
        }

        double lftTime = updTimer.milliseconds();
        updTimer.reset();
        //RobotLog.dd(TAG, "pose(11) is now: %s", drive.getLocalizer().getPoseEstimate().toString());

        double clawTime;
//        if(claw != null)
//        {
//            claw.update();
//        }
        updTimer.reset();
        // RobotLog.dd(TAG, "pose(12) is now: %s", drive.getLocalizer().getPoseEstimate().toString());

//        if(VERBOSE) { RobotLog.dd(TAG, "UPD BOT:%.2f LFT:%.2f CLW:%.2f", botTime, lftTime, clawTime); }
        ledUpdate();
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public void ledUpdate() {
        /*switch (pixelPieces){
            case 2:
                greenLED1.setState(false);
                redLED1.setState(false);
                greenLED2.setState(false);
                redLED2.setState(false);
                break;
            case 1:
            case 3:
                greenLED1.setState(false);
                redLED1.setState(true);
                greenLED2.setState(false);
                redLED2.setState(true);
                break;
            case 4:
                greenLED1.setState(true);
                redLED1.setState(false);
                greenLED2.setState(true);
                redLED2.setState(false);
                break;
            case 0:
                greenLED1.setState(true);
                redLED1.setState(true);
                greenLED2.setState(true);
                redLED2.setState(true);
                break;


        }*/
    }

  /* public void initArmMot() throws InterruptedException {

        arm.moveToCnt(2000,.02);

        Thread.sleep(1000);
        arm.setMode(STOP_AND_RESET_ENCODER);
        arm.setMode(RUN_USING_ENCODER);
        arm.moveToCnt(-300, 0.4);
        Thread.sleep(2000);
        arm.moveToCnt(-215, 0.1);
    }
    public void initParkMot() throws InterruptedException {
        park.moveToCnt(2000,.02);

        Thread.sleep(2000);
        park.setMode(STOP_AND_RESET_ENCODER);
        park.setMode(RUN_USING_ENCODER);
    }
    public void initClaw() {
        claw.setClawPos(1);
        RobotLog.dd(TAG, "claw open init");
    }
*/

    public void initShooter() throws InterruptedException {
        // shooterP.initPos();
    }

    public void initHummingBirds() {
        babyHummingbird.setPosition(0);
        hummingBird.setPosition(HUMMINGBIRDSTARTINGPOS);
    }

    // public void initSlides() throws InterruptedException {
    //    slides.initPos();
    // }


    //  public void setClawPos(double input){
    //         claw.setClawPos(input * WR_SENSE );
    //  }


    boolean extenderStopped = true;

    public void setExtenderPower(double input) {
//        if(input >= .1 && extenderMotor.getCurEnc() < EX_MAX || input <= -.1 && extenderMotor.getCurEnc() > EX_MIN){
//            extenderMotor.moveAtControlRate(input);
//            extenderStopped = false;
//        }else{
//            if(!extenderStopped) {
//                extenderMotor.moveAtControlRate(0);
//                extenderStopped = true;
//            }
//        }

    }

    final double DESIRED_DISTANCE = 13.0; //  this is how close the camera should get to the target (inches)
    double DESIRED_HEADING = 0;
    final double DESIRED_YAW = -5;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.04;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = .1;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 1;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.1;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    public VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.


    private void setManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Wait for the camera to be open, then use the controls
        RobotLog.dd(TAG, String.format("In Manual Exposure"));

        if (visionPortal == null) {
            return;
        }
        RobotLog.dd(TAG, "Valid VisionPortal");


        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            RobotLog.dd(TAG, "Camera not streaming");

            while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                Thread.sleep(20);
            }
            RobotLog.dd(TAG, "Camera Ready");

        }

        // Set camera controls unless we are stopping.
        if (!opMode.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            Thread.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            if (VERBOSE) {
                RobotLog.dd(TAG, String.format("Min Gain: %d, Max Gain: %d", gainControl.getMinGain(), gainControl.getMaxGain()));
            }
            Thread.sleep(20);
        }
    }

    private AprilTagSingleDetection desiredTag = null;
// Used to hold the data for a detected AprilTag

    private LinearOpMode opMode;

    public void initAprilTag(HardwareMap hardwareMap, LinearOpMode callerOpMode) {
        opMode = callerOpMode;
        setAlliance(Field.Alliance.valueOf(PreferenceMgr.getAllianceColor()));
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
            try {
                setManualExposure(5, 250);  // Use low exposure time to reduce motion blur
            } catch (Exception e) {
                RobotLog.dd(TAG, String.format("setManualExposure failed: %s", e.getMessage()));
            }

        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    public boolean tippedState() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        //RobotLog.dd(TAG, String.format("in tipped State"));

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection anyDetection : currentDetections) {

            // SDK 12+: detections can be single tags or tag clusters; only single tags have id/metadata.
            if (!(anyDetection instanceof AprilTagSingleDetection)) continue;
            AprilTagSingleDetection detection = (AprilTagSingleDetection) anyDetection;
            // The tag ID is available even when the tag is not in the tag library.
            // Tags 30-45 are NOT in the DECODE library (IDs 20-24), so metadata (and ftcPose)
            // will be null for them -- don't require it just to check the ID.
            {
                //  Check to see if we want to track towards this tag.
                RobotLog.dd(TAG, String.format("detected aprilTag: %d", detection.id));
                Field.Alliance alliance = getAlliance();
                if ((alliance == RED && (detection.id == RED_CELL_APRIL_TAG1_FIRST || detection.id == RED_CELL_APRIL_TAG2_FIRST || detection.id == RED_CELL_APRIL_TAG3_FIRST || detection.id == RED_CELL_APRIL_TAG4_FIRST || detection.id == RED_CELL_APRIL_TAG1 || detection.id == RED_CELL_APRIL_TAG2 || detection.id == RED_CELL_APRIL_TAG3 || detection.id == RED_CELL_APRIL_TAG4))) {
                    // Yes, we want to use this tag.
                    desiredTag = detection;
                    return true;

                    // don't look any further.
                } else if ((alliance == BLUE && (detection.id == BLUE_CELL_APRIL_TAG1_FIRST || detection.id == BLUE_CELL_APRIL_TAG2_FIRST || detection.id == BLUE_CELL_APRIL_TAG3_FIRST || detection.id == BLUE_CELL_APRIL_TAG4_FIRST || detection.id == BLUE_CELL_APRIL_TAG1 || detection.id == BLUE_CELL_APRIL_TAG2 || detection.id == BLUE_CELL_APRIL_TAG3 || detection.id == BLUE_CELL_APRIL_TAG4))) {
                    // Yes, we want to use this tag.
                    desiredTag = detection;
                    return true;
                }

            }
        }

//public  int armLevel;
//public  int slideLevel;


        //TODO: combine arm and slide together into a single level concept


        return false;
    }
}

