package com.qualcomm.robotcore.hardware;

import java.util.Locale;

/* MeepMeep stand-in for the real FTC SDK class (see FtcStandIns/build.gradle).  Only MeepMeep
 * uses this file; the robot uses the real one.  Keep it matching the real class exactly. */
public class PIDFCoefficients
{
    public double p;
    public double i;
    public double d;
    public double f;

    public PIDFCoefficients() {}

    public PIDFCoefficients(double p, double i, double d, double f)
    {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public PIDFCoefficients(PIDFCoefficients other)
    {
        this(other.p, other.i, other.d, other.f);
    }

    @Override
    public String toString()
    {
        return String.format(Locale.US, "p=%f i=%f d=%f f=%f", p, i, d, f);
    }
}
