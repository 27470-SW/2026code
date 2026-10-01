package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.image.ITD_Detector;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name="Camera Test", group="Test")
//@Disabled
public class CameraTest extends InitLinearOpMode // implements FtcMenu.MenuButtons
{
    private static final String TAG = "SJH_CT";

    public CameraTest()
    {
        RobotLog.dd(TAG, "CAMERA TEST");
    }

    //@SuppressWarnings("RedundantThrows")
    @Override
    public void runOpMode() //throws InterruptedException
    {
        RobotLog.dd(TAG, "initCommon");
        /*
        Initializes the data logger, gamepads, dashboard, Vuforia, OPENCV libraries.
         */
        useOpenCv = true;
        initCommon(this);

        if(CommonUtil.getInstance().hasCam)
        {
            camera = CommonUtil.camera;
        }
        else
        {
            RobotLog.dd(TAG, "Camera Not found by OpenCV or Vuforia");
        }

        RobotLog.dd(TAG, "Setting up Power Play Detector");
        ppowerDetector = new ITD_Detector();



        dashboard.displayText(0, "READY TO START");

        while(!isStarted() && !isStopRequested() && !CommonUtil.cameraOpened)
        {
            idle();
        }

        if(camera != null)
        {
            RobotLog.dd(TAG, "Setting image processing pipeline");
            camera.setPipeline(ppowerDetector);
        }

        ElapsedTime advTmr = new ElapsedTime();
        double advTime = 2.0;

        while(!isStarted() && !isStopRequested())
        {
            if(advTmr.seconds() > advTime)
            {
                advTmr.reset();
                ppowerDetector.advanceStage();
            }
        }


        waitForStart();

        dashboard.clearDisplay();

        RobotLog.dd(TAG, "Action SCAN_IMAGE");
        dashboard.displayText(0, "Press A to scan for the Team Element");

        /* Check whether the play button on the driver station has been pressed and stay in this loop
        until stop is pressed */
        while(opModeIsActive() && !isStopRequested())
        {
            /* find image only Camera Attached or detected */
            if(camera != null)
            {
                gpad1.update();
                boolean scanRequest = gpad1.just_pressed(ManagedGamepad.Button.A);

                if (scanRequest)
                {
                    countScanRequest++;

                    dashboard.clearDisplay();
                    dashboard.displayText(0, "Scanning for Element Started: attempt:  "
                                             + countScanRequest);

                    RobotLog.dd(TAG, "Scanning for Element Started: attempt: "
                                     + countScanRequest);

                    scanPos = ITD_Detector.Position.NONE;
                    scanBarcode();
                    ElapsedTime scanRequestTimer = new ElapsedTime();

                    while (opModeIsActive() &&
                           scanRequestTimer.seconds() < scanRequestDelay &&
                           (scanPos == ITD_Detector.Position.NONE)
                    )
                    {
                        /*after capturing the image, we want to detect the team element within the image*/
                        detect();
                        dashboard.displayText(1, "POS:  " + scanPos);
                        dashboard.displayText(2, "Number Of Contours Found: "
                                                 + numDetectedContours);
                        dashboard.displayText(3, "Timestamp: "
                                                 + ppowerDetector.getTimestamp());

                        if (numDetectedContours > 1)
                        {
                            dashboard.displayText(3, "Multiple Items Detected");
                        }
                        else if (numDetectedContours == 0)
                        {
                            dashboard.displayText(3, "No Team Element Detected");
                        }
                    }
                    RobotLog.dd(TAG, "Got position: " + scanPos);
                    camera.setPipeline(null);
                    ppowerDetector.saveImages();
                    ppowerDetector.reset();
                }
            }
        }

        stopMode();
    }

    private void stopMode()
    {
        if(camera != null)
        {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }

    /* Read Camera Calibrations from Vuforia and print them out to the logger
     *  Since it's related to the camera we'll leave them in here
    */

    private void scanBarcode()
    {
        RobotLog.dd(TAG, "scanBarcode - setting pipeline");

        /* Start the camera to start capturing images*/
        if(camera != null)
        {
            camera.setPipeline(ppowerDetector);
        }
    }

    private void detect()
    {
        scanPos = ppowerDetector.getPos();
        numDetectedContours = ppowerDetector.getNumContours();
    }

    private ITD_Detector ppowerDetector;

    private OpenCvCamera camera;
    private double[] centerHSV;
    private static final double  scanRequestDelay = 1.0;
    private int countScanRequest = 0;

    private ITD_Detector.Position scanPos = ITD_Detector.Position.NONE;
    private int numDetectedContours = 0;
}