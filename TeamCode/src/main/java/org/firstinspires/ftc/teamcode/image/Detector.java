package org.firstinspires.ftc.teamcode.image;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Calendar;
import java.util.Locale;

public abstract class Detector extends OpenCvPipeline
{
    protected String name = "Detector";

    protected static String dateStr;

    private final static String TAG = "SJH_Detector";

    protected static int imgNum = 0;

    protected boolean viewportPaused = false;

    protected boolean crop = true;

    protected boolean logging = true;

    /* package-private */ Mat srcImg = new Mat();

    Detector()
    {
        Calendar cal = Calendar.getInstance();
        int dom = cal.get(Calendar.DATE);
        int mon = cal.get(Calendar.MONTH) + 1;
        int yr  = cal.get(Calendar.YEAR);
        int hr  = cal.get(Calendar.HOUR_OF_DAY);
        int min = cal.get(Calendar.MINUTE);
        dateStr = String.format(Locale.US,"%4d%02d%02d_%02d%02d", yr, mon, dom, hr, min);
    }

    public void logTelemetry()
    {
    }

    public void logDebug()
    {
    }

    public void setName(String name)
    {
        this.name = name;
    }

    public void setPaused(boolean paused)
    {
        viewportPaused = paused;
    }

    public void setCrop(boolean crop) { this.crop = crop; }

    public void setLogging(boolean enableLogging) { logging = enableLogging; }

    public String getName()
    {
        return name;
    }

    public void reset() {}

    public void saveImages() {}

    public void saveImage(Mat img, String imgTag)
    {
        if(img == null) return;
        String fileName = imgTag + "_" + imgNum++ + "_" + dateStr + ".bmp";

        Bitmap bmp = null;
        try
        {
            bmp = Bitmap.createBitmap(img.cols(), img.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(img, bmp);
        }
        catch (Exception e)
        {
            RobotLog.ee(TAG, e.getMessage());
        }

        String directoryPath  = Environment.getExternalStorageDirectory().getPath() +
                                        "/FIRST/DataLogger";
        String filePath       = directoryPath + "/" + fileName ;

        File dest = new File(filePath);
        FileOutputStream out = null;
        try
        {
            out = new FileOutputStream(dest);
            if(bmp != null)
            {
                // bmp is your Bitmap instance
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out);
            }
            // PNG is a lossless format, the compression factor (100) is ignored

        }
        catch (Exception e)
        {
            e.printStackTrace();
            RobotLog.ee(TAG, e.getMessage());
        }
        finally
        {
            try
            {
                if (out != null)
                {
                    out.close();
                    RobotLog.ii(TAG, "ImageSaved: " + fileName);
                }
            }
            catch (IOException e)
            {
                RobotLog.ee(TAG, e.getMessage());
                e.printStackTrace();
            }
        }
    }
}
