package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.MecanumTeleop;

@Config
@TeleOp(name = "TestMecanum", group="Test")
public class MecanumTest extends MecanumTeleop {

    @Override
    public void initPreStart() throws InterruptedException{
        VERBOSE = true;
        super.initPreStart();
    }
}
