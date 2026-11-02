package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;


public class Hummingbird {

    public Hummingbird(HardwareMap map, String servoH1Name, String servoH2Name, double servoH1_0, double servoH2_0)
    {
        this.hwMap = map;
        try {
            servoH1 = hwMap.get(Servo.class, servoH1Name);
            servoH2 = hwMap.get(Servo.class, servoH2Name);
        }catch(Exception e){
            RobotLog.ee(TAG, "Unable to find hummingbird");
        }
        //percent = 0.5;
      //  onIntake = 1;
      //  offIntake = 0.5;
        servoH1Zero = servoH1_0;
        servoH2Zero = servoH2_0;


    }
    public void init (double pos){
        setPosition(pos);
    }
    public void setPosition(double pos){
        percent = pos;



        if(servoH1 == null) return;
        servoH1.setPosition(getRealZero(pos, servoH1Zero));
        RobotLog.dd(TAG, "Percent:%f POS:%f", percent, pos);
        pos = 1-pos;
        if(servoH2 == null) return;
        servoH2.setPosition(getRealZero(pos, servoH2Zero));
        RobotLog.dd(TAG, "Percent:%f POS:%f", percent, pos);

    }
    public void moveHummingbirdLittle(double amount){
        percent = percent + amount;
        setPosition(percent);
        RobotLog.dd(TAG, "moving to position: %f", percent);
    }

    public double getRealZero(double pos, double zero){

        return zero + pos;
    }





    public void update(){

        //setIntakePower(percent);

    }





    public double percent;
    private static final String TAG = "servoIntake";
    public Servo servoH2;
    public Servo servoH1;
    public  double servoH2Zero;
    public double servoH1Zero;
    protected HardwareMap hwMap;
    public double onIntake;
    public double offIntake;
    public boolean Triggr;
    public int K1R2;
}
