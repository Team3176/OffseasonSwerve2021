/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class SwervePodConstants {

    public static final double WHEEL_DIAMETER = 3.25; //Inches
    public static final double DRIVE_GEAR_RATIO = (17.0/54.0);
    public static final double UNKNOWN_GEAR_RATIO = (30.0/48.0); //Might be its inverse

    public static final double ENCODER_UNITS = 4096;

    //FUTURE CLEANING: Turn these random, hard coded ratios into constants
    public static final double REV_2_FT = DRIVE_GEAR_RATIO * UNKNOWN_GEAR_RATIO * (WHEEL_DIAMETER*Math.PI) * (1.0/12.0);
    public static final double FPS_2_RPM = (1.0/REV_2_FT) * 16.0;
    //public static final double FPS_2_RPM = ((1.0/DRIVE_GEAR_RATIO)* (1.0/UNKNOWN_GEAR_RATIO) * (1.0/(WHEEL_DIAMETER*Math.PI)))  * 12.0 * 16.0;
    
    public static final double[] DRIVE_PID_OFFSEASON_OFFSETS = {
        /* kP */    0.000095,
        /* kI */    0.0000009325,
        /* kD */    0.0,
        /* kF */    0.0,    //Feed forward gain constant
        /* I-Zne */ 0.0     //The range of error for kI to take affect (like a reverse deadband)
    };

    public static final double[][] SPIN_PID_CONFIG = {
        /* kP */    {4.5, 1.0, 1.0, 4.5},
        /* kI */    {0.0023, 0.0, 0.0, 0.0023},
        /* kD */    {190.0, 400.0, 400.0, 190.0},
        /* kF */    {0.00001, 0.00001, 0.00001, 0.00001}    //Feed forward gain constant
    };

    public static final double[] OFFSETS = {2000};
    //public static final double[] OFFSETS = {2000, 2000, 2000, 2000}

    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final double DRIVE_RAMP_RATE = 0.2;
}
