package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.EnumMap;
import java.util.Locale;

public class ManagedGamepad
{
    private static final String TAG = "SJH_MGP";
    private static int cycle = 0;
    private final EnumMap<Button, Boolean> current       = new EnumMap<>(Button.class);
    private final EnumMap<Button, Boolean> previous      = new EnumMap<>(Button.class);
    private final EnumMap<Button, Boolean> just_pressed  = new EnumMap<>(Button.class);
    private final EnumMap<Button, Boolean> just_released = new EnumMap<>(Button.class);
    private final EnumMap<AnalogInput, Double>  scale    = new EnumMap<>(AnalogInput.class);

    private final HalDashboard dashboard;

    private final Gamepad gamepad;
    Gamepad.RumbleEffect customRumbleEffect1;    // Use to build a custom rumble sequence.
    Gamepad.RumbleEffect customRumbleEffect2;    // Use to build a custom rumble sequence.

    public ManagedGamepad(Gamepad gamepad)
    {
        this.gamepad = gamepad;
        dashboard = CommonUtil.getInstance().getDashboard();
        init();
    }

    private void init()
    {
        for(Button b : Button.values())
        {
            current.put(b, false);
            previous.put(b, false);
            just_pressed.put(b, false);
        }

        for(AnalogInput a : AnalogInput.values())
        {
            scale.put(a, 0.0);
        }
        /* TODO: make a custom builder function to make custom effects */
        customRumbleEffect1 = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 500)  //  Pause for 300 mSec
                .addStep(0.0, 1.0, 500)
                .addStep(1.0, 0.0, 500)
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 500)  //  Pause for 300 mSec
                .addStep(0.0, 1.0, 500)
                .addStep(1.0, 0.0, 500)
                .addStep(0.0, 1.0, 500)
                .addStep(0.0, 0.0, 500)    //wait for end game to start
                .addStep(1.0, 1.0, 1000)
                .build();

        customRumbleEffect2 = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 250 ms at 15 seconds till end of game
                .addStep(0.0, 0.0, 4500)  //  Pause until 10 seconds
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 3000)  //  Pause until 6.5 seconds
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 250 mSec
                .addStep(0.0, 0.0, 1250)  //  Pause until 4.75 seconds
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 1000)  //  Pause until 3.25
                .addStep(1.0, 1.0, 1000)  //  Rumble both sides for 0.5 seconds
                .addStep(0.0, 0.0, 750)  //  Pause until 1.5 seconds
                .addStep(1.0, 1.0, 1500)  //  Rumble both sides until end of game
                .build();
    }

    private void update_just_pressed()
    {
        for(int i = 0; i < Button.values().length ; i++)
        {
            Button b = Button.values()[i];
            Boolean curB = current.get(b);
            Boolean prvB = previous.get(b);
            boolean cur = false;
            boolean prv = false;
            if (curB != null) cur = curB;
            if (prvB != null) prv = prvB;
            just_pressed.put(b,   cur && !prv);
            just_released.put(b, !cur &&  prv);
            previous.put(b, current.get(b));
        }
    }

    public void update()
    {
        if(gamepad == null)
        {
            if(cycle == 0)
            {
                RobotLog.ee(TAG, "ERROR: No gamepad");
            }
            cycle++;
            return;
        }

        current.put(Button.A,         gamepad.a);
        current.put(Button.B,         gamepad.b);
        current.put(Button.X,         gamepad.x);
        current.put(Button.Y,         gamepad.y);
        current.put(Button.D_UP,      gamepad.dpad_up);
        current.put(Button.D_DOWN,    gamepad.dpad_down);
        current.put(Button.D_LEFT,    gamepad.dpad_left);
        current.put(Button.D_RIGHT,   gamepad.dpad_right);
        current.put(Button.L_BUMP,    gamepad.left_bumper);
        current.put(Button.R_BUMP,    gamepad.right_bumper);
        current.put(Button.L_TRIGGER, gamepad.left_trigger  > 0.1);
        current.put(Button.R_TRIGGER, gamepad.right_trigger > 0.1);
        current.put(Button.L_STICK_BUTTON, gamepad.left_stick_button);
        current.put(Button.R_STICK_BUTTON, gamepad.right_stick_button);
        current.put(Button.START,     gamepad.start);
        scale.put(AnalogInput.L_STICK_X,     (double)gamepad.left_stick_x);
        scale.put(AnalogInput.L_STICK_Y,     (double)gamepad.left_stick_y);
        scale.put(AnalogInput.R_STICK_X,     (double)gamepad.right_stick_x);
        scale.put(AnalogInput.R_STICK_Y,     (double)gamepad.right_stick_y);
        scale.put(AnalogInput.L_TRIGGER_VAL, (double)gamepad.left_trigger);
        scale.put(AnalogInput.R_TRIGGER_VAL, (double)gamepad.right_trigger);

        update_just_pressed();
    }

    public boolean just_pressed(Button b)
    {
        final Boolean ret = just_pressed.get(b);
        return ret == null ? false : ret;
    }

    @SuppressWarnings("unused")
    public boolean just_released(Button b)
    {
        final Boolean ret = just_released.get(b);
        return ret == null ? false : ret;
    }

    public boolean pressed(Button b)
    {
        final Boolean ret = current.get(b);
        return ret == null ? false : ret;
    }

    public double value(AnalogInput a)
    {
        final Double ret = scale.get(a);
        return ret == null ? 0.0 : ret;
    }

    public void customRumble(int i)
    {
        if(1 == i)
        {
            gamepad.runRumbleEffect(customRumbleEffect1);
        }
        else
        {
            gamepad.runRumbleEffect(customRumbleEffect2);
        }
    }

    public void rumbleBlips(int blipCount)
    {
        gamepad.rumbleBlips(blipCount);
    }

    public boolean gamePadRumbling()
    {
        return (gamepad.isRumbling());
    }

    public void gamePadRumbleForDuration(double leftMotorPwr, double rightMotorPwr, int duration)
    {
        gamePadRumbleForDuration(leftMotorPwr,rightMotorPwr, duration);
    }

    public void gamePadStopRumbling()
    {
        gamepad.stopRumble();
    }


    @SuppressWarnings("unused")
    public void log(int idx)
    {
        StringBuilder pressed = new StringBuilder("1_");
        for(Button b : Button.values())
        {
            if (pressed(b))
            {
                pressed.append(b);
            }
        }
        dashboard.displayText(idx, pressed.toString());
        for(AnalogInput a : AnalogInput.values())
        {
            idx +=2;
            dashboard.displayText(idx,
                String.format(Locale.US, "%s %4.3f", a, scale.get(a)));
        }
    }

    public enum Button
    {
        A,
        B,
        X,
        Y,
        D_UP,
        D_DOWN,
        D_LEFT,
        D_RIGHT,
        L_BUMP,
        R_BUMP,
        L_TRIGGER,
        R_TRIGGER,
        L_STICK_BUTTON,
        R_STICK_BUTTON,
        START
    }

    public enum AnalogInput
    {
        L_STICK_X,
        L_STICK_Y,
        R_STICK_X,
        R_STICK_Y,
        L_TRIGGER_VAL,
        R_TRIGGER_VAL
    }
}
