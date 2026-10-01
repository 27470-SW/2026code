package org.firstinspires.ftc.teamcode.image;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.PreferenceMgr;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;

@SuppressWarnings("unused")
public class ITD_Detector extends Detector
{
    public enum Position
    {
        LEFT, CENTER, RIGHT, NONE
    }
    public enum Color
    {
        RED,
        BLUE
    }
    protected static Field.Alliance alliance;
    public static ITD_Detector.Position foundPosition = ITD_Detector.Position.NONE;
    private long timestamp = 0L;
    private final ITD_Pipeline gpl;

    private static final String TAG = "SJH_PPD";

    /*Disable Saving of images for competition if it is slowing down auton*/
    @SuppressWarnings("FieldCanBeLocal")
    private final boolean saveSizedImage  = true;
    @SuppressWarnings("FieldCanBeLocal")
    private final boolean saveOriginalImage = true;

    private int numContoursFound;

    @SuppressWarnings("FieldCanBeLocal")
    private final boolean saveMaskedImage = true;
    @SuppressWarnings("FieldCanBeLocal")
    private final boolean saveThreshImage = true;

    private final Mat showImg = new Mat();
    private Mat sizedImage;
    private final Mat showSzdImg = new Mat();
    private Rect rawCtrRect;
    private Rect szdCtrRect;
    private Point szdLctr;
    private Point szdRCtr;
    private Rect thrdsRect;
    private final Scalar ctrRectColor = new Scalar(0, 255, 0);
    private final Scalar contourColorWHITE = new Scalar(255, 255, 255);


    private ITD_Pipeline.PipeLineStage stageToRenderToViewport =
      ITD_Pipeline.PipeLineStage.RAW;
    private final ITD_Pipeline.PipeLineStage[] stages = ITD_Pipeline.PipeLineStage.values();

    @SuppressWarnings("WeakerAccess")
    public ITD_Detector()
    {
        name = "FfDetector";
        alliance = Field.Alliance.valueOf(PreferenceMgr.getAllianceColor());
        gpl = new ITD_Pipeline();
    }

    public ITD_Detector(String name)
    {
        this();
        gpl.setName(name);
    }

    public void logTelemetry()
    {
        CommonUtil.getInstance().getDashboard()
                .displayText(4,"Location Detected: " + foundPosition);
    }

    public void logDebug()
    {
        RobotLog.ii(TAG, " Ff Location Detected %s", foundPosition);
    }

    public ITD_Detector.Position getPos()
    {
        return foundPosition;
    }

    public int getNumContours()
    {
        return (numContoursFound);
    }

    public boolean isPipeLineRaw()
    {
        boolean isPipeLineRawStage = true;

        if (stageToRenderToViewport != ITD_Pipeline.PipeLineStage.RAW)
        {
            isPipeLineRawStage = false;
        }
        return (isPipeLineRawStage);
    }


    public long getTimestamp() { return timestamp; }

    public void reset()
    {
        timestamp = 0L;
        numContoursFound = 0;
        foundPosition = Position.NONE;
    }

    public void setLogging(boolean enableLogging)
    {
        logging = enableLogging;
        if(gpl != null) gpl.setLogging(logging);
    }

    public Mat processFrame(Mat mat)
    {
        RobotLog.ii(TAG, "FfDetector.processFrame()");
        mat.copyTo(srcImg);
        findPosition();
        String text;
        Point position = new Point(5,20);
        Scalar color = new Scalar(255, 0, 255);
        Scalar textColorOrange = new Scalar(255, 165, 0);
        Scalar textColorGreen = new Scalar(1, 68, 33);
        Scalar textColorPurple = new Scalar(102, 51, 153);
        Scalar textColorBlack = new Scalar(0, 0, 0);
        int font = Imgproc.FONT_HERSHEY_SIMPLEX;
        int scale = 1;
        int thickness = 2;

        Mat outImg = srcImg;

        if(!viewportPaused)
        {
            //RobotLog.ii(TAG, "FfDetector.viewportpaused()");
            switch (stageToRenderToViewport)
            {
                case RAW:
                    RobotLog.ii(TAG, "FfDetector.raw()");
                    /*show the image*/
                    srcImg.copyTo(showImg);
                    outImg = showImg;
                    /*draw a rectangle for the crop box*/
                    Imgproc.rectangle(outImg, gpl.roiRect(), ctrRectColor, 4);
                    Imgproc.rectangle(outImg, rawCtrRect, ctrRectColor, 4);
                    text = "RAW " +  showImg.cols() + " " + showImg.rows();

                    /*Adding text to the image*/
                    Imgproc.putText(outImg, text, position, font, scale, color, thickness);
                    break;

                case SIZE:
                    /*show the image*/
                    RobotLog.ii(TAG, "FfDetector.size()");
                    gpl.resizeImageOutput().copyTo(showSzdImg);
                    outImg = showSzdImg;
                    /*draw the crop box for the sized image*/
                    Imgproc.rectangle(outImg, szdCtrRect, ctrRectColor, 4);
                    Imgproc.rectangle(outImg, thrdsRect, ctrRectColor, 4);
                    Imgproc.line(outImg, szdLctr, szdRCtr, ctrRectColor, 1);

                    text = "Sized" +  showSzdImg.cols() + " " + showSzdImg.rows() ;

                    /*Adding text to the image*/
                    Imgproc.putText(outImg, text, position, font, scale, color, thickness);

                    break;

                case HSV:
                    RobotLog.ii(TAG, "FfDetector.HSV()");
                    gpl.resizeImageOutput().copyTo(showSzdImg);
                    outImg = showSzdImg;
                    /*If the park location stored is LEFT, CENTER or RIGHT, show the image and output the corresponding color*/
                    if(foundPosition != Position.NONE)
                    {
                        outImg = gpl.hsvThresholdOutput(Color.RED.ordinal());

                    }

                    text = "HSV";
                    Imgproc.putText(outImg, text, position, font, scale, textColorOrange, thickness);
                    break;

                case ERODE:
                    gpl.resizeImageOutput().copyTo(showSzdImg);
                    RobotLog.ii(TAG, "FfDetector.ERODE()");
                    outImg = showSzdImg;
                    /*If the park location stored is LEFT, CENTER or RIGHT, show the image and output ERODE followed by the color*/
                    if(foundPosition != Position.NONE)
                    {
                        outImg = gpl.cvErodeOutput(Color.RED.ordinal());

                    }

                    text = "CV ERODE";
                    Imgproc.putText(outImg, text, position, font, scale, textColorOrange, thickness);
                    break;
                case DILATE:
                    gpl.resizeImageOutput().copyTo(showSzdImg);
                    RobotLog.ii(TAG, "FfDetector.DILATE()");
                    outImg = showSzdImg;
                    text = "Dilate";
                    Imgproc.putText(outImg, text, position, font, scale, color, thickness);
                    if(foundPosition != Position.NONE)
                    {
                        outImg = gpl.cvDilateOutput(Color.RED.ordinal());

                    }
                    text = "Dilate";

                    Imgproc.putText(outImg, text, position, font, scale, textColorOrange, thickness);
                    break;

                case CONTOURS:
                    gpl.resizeImageOutput().copyTo(showSzdImg);
                    RobotLog.ii(TAG, "FfDetector.CONTOURS()");
                    outImg = showSzdImg;
                    /* If the park location stored is LEFT, CENTER or RIGHT, draw the contours for
                       the corresponding color. The outline will be a set to a color defined with RGB values */
                    if(foundPosition != Position.NONE)
                    {
                        Imgproc.drawContours(outImg, gpl.findContoursOutput(Color.RED.ordinal()), -1, contourColorWHITE, 4, 8);
                    }
                    text = "Draw Contours";
                    Imgproc.putText(outImg, text, position, font, scale, color, thickness);
                    break;

                case CONVEX:
                    gpl.resizeImageOutput().copyTo(showSzdImg);
                    RobotLog.ii(TAG, "FfDetector.CONVEX()");
                    outImg = showSzdImg;
                    if(foundPosition != Position.NONE)
                    {
                        Imgproc.drawContours(outImg, gpl.convexHullsOutput(Color.RED.ordinal()), -1, contourColorWHITE, 4, 8);
                    }

                    text = "CONVEX HULLS";
                    Imgproc.putText(outImg, text, position, font, scale, color, thickness);
                    break;

                case FILTER:
                    gpl.resizeImageOutput().copyTo(showSzdImg);
                    RobotLog.ii(TAG, "FfDetector.FILTER()");
                    outImg = showSzdImg;

                    if(foundPosition != Position.NONE)
                    {
                        Imgproc.drawContours(outImg, gpl.filterContoursOutput(Color.RED.ordinal()), -1, contourColorWHITE, 4, 8);
                    }

                    /*Adding text to the image*/
                    text = "Contours: " + numContoursFound;
                    Imgproc.putText(outImg, text, position, font, scale, color, thickness);

                    break;
            }
        }

        return outImg;
    }
    /* @saveImages -
    *  Description: Spawns 3 threads and saves Images to SDCard "/sdcard/EasyOpenCV" on control hub
    *               Images are saved post processing of filter and after Park Position is detected
    */
    public void saveImages()
    {
        RobotLog.ii(TAG, "FfDetector.saving images...()");
        if (saveOriginalImage)
        {
            Thread t1 = new Thread(() -> {
                //saveImage(showImg, "raw");
                String fileName = "raw" + "_" + imgNum++ + "_" + dateStr;
                saveMatToDisk(srcImg, fileName);
            }, "saveOriginalImageThread");
            t1.setPriority(Thread.MIN_PRIORITY + 1);
            t1.start();
        }

        if (saveSizedImage)
        {
            Thread t2 = new Thread(() ->
            {
                //saveImage(sizedImage, "sized");
                String fileName = "sized" + "_" + imgNum++ + "_" + dateStr;
                saveMatToDisk(sizedImage, fileName);
            }, "saveSizedImageThread");
            t2.setPriority(Thread.MIN_PRIORITY + 2);
            t2.start();
        }

        if (saveThreshImage)
        {
            Mat thrshImg;
            thrshImg = gpl.hsvThresholdOutput(Color.RED.ordinal());

            Thread t3 = new Thread(() -> {
                String fileName = "minThresh" + "_" + imgNum++ + "_" + dateStr;
                saveMatToDisk(thrshImg, fileName);}, "saveThreshImageThread");
            t3.setPriority(Thread.MIN_PRIORITY + 3);
            t3.start();

        }
    }
    /* Used during competition in the Camera Stream to position the camera crop guidelines */
    public void toggleStage()
    {
        if(stageToRenderToViewport == ITD_Pipeline.PipeLineStage.RAW)
        {
            stageToRenderToViewport = ITD_Pipeline.PipeLineStage.SIZE;
        }
        else
        {
            stageToRenderToViewport = ITD_Pipeline.PipeLineStage.RAW;
        }
    }

    public void advanceStage()
    {
        // Set the stage to RAW to display the full image
        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }
    /* When image is tapped on driver station when in camera stream,
       It advances the pipeline stage to show the found park position filtered image */
    public void onViewportTapped()
    {
        RobotLog.ii(TAG, "FfDetector.onViewportTapped()");
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        advanceStage();
    }

    /* Init invoked by the OpenCVPipeline */
    public void init(Mat mat)
    {
        if(logging)
        {
            RobotLog.ii(TAG, "FfDetector.init()");
        }
        srcImg = mat.clone();

        gpl.setCrop(crop);
        gpl.init(mat);

        Rect roiRect = gpl.roiRect();

        /* Green guidelines in RAW image that shows the crop region of interest */
        rawCtrRect = new Rect(new Point((double)(mat.cols())/2 - 5, (double)(mat.rows())/2 - 5), new Point((double)(mat.cols())/2 + 5, (double)mat.rows()/2 + 5));
        Mat szdMat = gpl.resizeImageOutput();
        /* Rectangles on sized and cropped image that divides the image in 3rds */
        thrdsRect = new Rect(new Point((double)(szdMat.cols())/3, 0), new Point(2*szdMat.cols()/3, roiRect.height-1));
        /* Center Focal point Rectangle to indicate the true center of the camera image */
        szdCtrRect = new Rect(new Point(szdMat.cols()/2 - 5, szdMat.rows()/2 - 5), new Point(szdMat.cols()/2 + 5, szdMat.rows()/2 + 5));
        szdLctr = new Point(0.0,(double)(szdMat.rows())/2);
        szdRCtr = new Point(szdMat.cols()-1,(double)szdMat.rows()/2);
    }
    /* Detect the signal sleeve color
       Runs the image filter 3 times, stores the number of contours found for each color, then
       determines the corresponding park position
    */
    private void findPosition()
    {
        timestamp = System.currentTimeMillis();

        if (srcImg == null)
        {
            RobotLog.ee(TAG, "FfDetector.findPosition() null image");
        }

        gpl.sizeSource(srcImg);

        sizedImage = gpl.resizeImageOutput();

        ArrayList<MatOfPoint> contours;
        if (alliance == Field.Alliance.RED)
        {
            // Apply image filters for RED color detection
            gpl.applyImageFilters(sizedImage, Color.RED);
        }
        else
        {
            // Apply image filters for BLUE color detection
            gpl.applyImageFilters(sizedImage, Color.BLUE);
        }



        contours = gpl.filterContoursOutput(0);
        Iterator<MatOfPoint> each = contours.iterator();

        Rect bounded_box;

        foundPosition = Position.NONE;

        if (logging) RobotLog.dd(TAG, "Processing %d contours", contours.size());
        numContoursFound = contours.size();

        int contourX = -1;

        if (contours.size() == 0)
        {
            if (logging) RobotLog.dd(TAG, "No Contours found");
            foundPosition = Position.NONE;
        }
        else
        {
            if (contours.size() > 1)
            {
                if (logging) RobotLog.ee(TAG, " found %d contours, evaluating largest",
                        contours.size());
            }

            double maxA = 0.0;
            for (MatOfPoint pts : contours)
            {
                MatOfPoint contour = each.next();
                bounded_box = Imgproc.boundingRect(contour);

                /* If multiple contours are found, use the largest one.
                 * As long as the image filter does not pick up anything larger than the team element, this will work */
                if (bounded_box.area() > maxA)
                {
                    maxA = bounded_box.area();
                    /* Use the center x position of the contour instead of the top left corner */
                    contourX = bounded_box.x + bounded_box.width / 2;
                }
                if (logging) RobotLog.dd(TAG, "ContourX:" + contourX);

                if (contourX > (int) ((float) sizedImage.width() * 2.0f / 3.0f))
                {
                    foundPosition = Position.RIGHT;
                }
                else if (contourX > (int) ((float) sizedImage.width() / 3.0f))
                {
                    foundPosition = Position.CENTER;
                }
                else
                {
                    foundPosition = Position.LEFT;
                }

                if (logging) RobotLog.dd(TAG, "Found pos=%s", foundPosition);
            }
        }
    }
}
