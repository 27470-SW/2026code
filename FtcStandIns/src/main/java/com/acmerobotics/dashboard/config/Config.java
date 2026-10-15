package com.acmerobotics.dashboard.config;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/* MeepMeep stand-in for FTC Dashboard's @Config (see FtcStandIns/build.gradle).  It must stay
 * RUNTIME: that is what tells the real FTC Dashboard on the robot to find @Config classes. */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface Config
{
    String value() default "";
}
