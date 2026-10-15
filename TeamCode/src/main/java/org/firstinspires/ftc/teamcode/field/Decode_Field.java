package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Point2d;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.field.Field.halfField;
import static org.firstinspires.ftc.teamcode.field.Field.halfTile;
import static org.firstinspires.ftc.teamcode.field.Field.mmTargetHeight;
import static org.firstinspires.ftc.teamcode.field.Field.oneAndHalfTile;

/* The field image/AprilTag locations (Android only).  The field sizes and menu choices
 * are in Field, in SharedCode. */
@SuppressWarnings({"unused", "WeakerAccess"})
public class Decode_Field
{
    public Decode_Field()
    {
        this.assetName = "PowerPlay";
        setImageNames();
        setImageLocations();
        imgTransformMap = createPosMap(this.trackableNames, locationsOnField);
        setHasVuMarks(false);
    }

    private final String         assetName;
            List<String>         trackableNames   = new ArrayList<>();
            List<OpenGLMatrix>   locationsOnField = new ArrayList<>();

    public static final float IMAGE_Z = mmTargetHeight;

    private static final float scale = (float) Units.MM_PER_INCH;

    //Note: asset file has 304mm x 224mm (12"x8.8")for RR !?!?
    //Need to figure out what xml coordinates really mean
//    public static int target_width  = 127 * 2;
//    public static int target_height = 92 * 2;
    //It turns out that we really need the trackable page dimensions in this case
    //So use 8.5"x11" paper in landscape =
    public static int target_width  = 280;
    public static int target_height = 216;

    static float[] scaleArr(float[] inArr)
    {
        float[] outArr = {inArr[0], inArr[1], inArr[2]};
        for (int i =0; i<inArr.length; ++i)
        {
            outArr[i]*= scale;
        }
        return outArr;
    }

    static OpenGLMatrix genMatrix(float[] pos, float[] rot)
    {
        return OpenGLMatrix
                .translation(pos[0], pos[1], pos[2])
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,
                        rot[0], rot[1], rot[2]));
    }

    public String getAssetName()    { return assetName; }
    public List<String> getImageNames() { return trackableNames; }
    public List<OpenGLMatrix> getImageTransforms() { return locationsOnField; }
    OpenGLMatrix getAssetTransform(String name)
    {
        return imgTransformMap.get(name);
    }

    @SuppressWarnings("SameParameterValue")
    void setHasVuMarks(boolean hasVuMarks)
    {
        this.hasVuMarks = hasVuMarks;
    }
    public boolean hasVuMarks() {return hasVuMarks;}
    private boolean hasVuMarks = false;

    private static Map<String, OpenGLMatrix> imgTransformMap;

    private static Map<String, OpenGLMatrix> createPosMap(List<String> names,
                                                          List<OpenGLMatrix> transforms)
    {
        Map<String, OpenGLMatrix> map = new HashMap<>();
        for (int i = 0; i < names.size() && i < transforms.size(); ++i)
        {
            map.put(names.get(i), transforms.get(i));
        }
        return map;
    }
	

    //Point naming key:
    //1st char: R=Red, B=Blue
    //2nd char: L=Left start, R=Right start (viewed from red side - along field X)
    //3rd-4th chars: Pt description

    private static final String TAG = " SJH_RFD";

    private static final int BLUE  = 0;
    private static final int RED   = 1;
    private static final int STRT1 = 0;
    private static final int STRT2 = 1;
    private static final int LEFT  = 0;
    private static final int CNTR  = 1;
    private static final int RGHT  = 2;

    //Red Route > Left
    //Red (Left/Right) Start Points
    public static final Point2d ROS1 = new Point2d("ROS1", -61.5,  -47);
    public static final Point2d RIS1 = new Point2d("RIS1", -61.5,  -24);

    //Red (Left/Right) Scan Pt's (Image Scan Pt)
    public static final Point2d ROSP = new Point2d("ROSP", -61.45,  -47);
    public static final Point2d RISP = new Point2d("RISP", -61.45,  -24);

    private static final int ALNC_RED = 0;
    private static final int ALNC_BLU = 1;
    private static final int STRT_ONE = 0;
    private static final int STRT_TWO = 1;
    private static final int STN_LEFT = 0;
    private static final int STN_CNTR = 1;
    private static final int STN_RGHT = 2;

    private static final String ASSET_NAME = "PowerPlay";

    void setImageNames()
    {
       trackableNames.add("BlueStorage");
       trackableNames.add("BlueAllianceWall");
       trackableNames.add("RedStorage");
       trackableNames.add("RedAllianceWall");
    }

    void setImageLocations()
    {
        float[][] TRACKABLE_POS = {
                scaleArr(new float[]{-halfField,  oneAndHalfTile, IMAGE_Z}),
                scaleArr(new float[]{ halfTile,   halfField,      IMAGE_Z}),
                scaleArr(new float[]{-halfField, -oneAndHalfTile, IMAGE_Z}),
                scaleArr(new float[]{ halfTile,  -halfField,      IMAGE_Z})
        };

        locationsOnField.add(genMatrix(TRACKABLE_POS[0], new float[]{  90.0f,    0.0f,   90.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[1], new float[]{  90.0f,    0.0f,    0.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[2], new float[]{  90.0f,    0.0f,   90.0f}));
        locationsOnField.add(genMatrix(TRACKABLE_POS[3], new float[]{  90.0f,    0.0f,  180.0f}));
    }
}
