package frc.robot.constants;

public final class SwervePodConstants {

    private static final double WHEEL_DIAMETER = 3.25; //Inches
    private static final double DRIVE_GEAR_RATIO = (17.0/54.0);
    private static final double UNKNOWN_GEAR_RATIO = (30.0/48.0); //Might be its inverse

    public static final double ENCODER_UNITS = 4096;

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
        /* kP */    {3.0, 1.0, 1.0, 4.5},
        /* kI */    {0.0023, 0.0, 0.0, 0.0023},
        /* kD */    {190.0, 400.0, 400.0, 190.0},
        /* kF */    {0.00001, 0.00001, 0.00001, 0.00001}    //Feed forward gain constant
    };

    public static final int[] OFFSETS = {0};
    //public static final int[] OFFSETS = {2000, 2000, 2000, 2000}

    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final double DRIVE_RAMP_RATE = 0.2;
    public static final double DRIVE_SPEED_MAX_EMPIRICAL_FPS = 13.0;
}
