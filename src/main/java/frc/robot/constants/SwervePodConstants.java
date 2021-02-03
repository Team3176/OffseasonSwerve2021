package frc.robot.constants;

public final class SwervePodConstants {

    private static final double WHEEL_DIAMETER = 3.25; //Inches
    private static final double WHEEL_CIRCUMFERENCE = 3.25 * Math.PI;
    private static final double UNKNOWN_GEAR_RATIO = (30.0/48.0); //Might be its inverse

    private static final double SPIN_GEAR_RATIO = 70.0/1.0;
    private static final double DRIVE_GEAR_RATIO = (54.0 / 14.0) * (48.0 / 30.0);
    // private static final double 
    // SPIN
    // 140 rev/min
    // after 70 to 1 --> 2 rev/min
    // encoder reads 4096 * 2
    // wheel spins 2 rev/min
    
    // DRIVE
    // 54 / 14
    // 48 / 30
    // So 6.17... motor revolutions means 1 wheel rotation



  

    public static final double ENCODER_UNITS = 4096;
    
    // What is this for
    public static final double[] DRIVE_PID_OFFSEASON_OFFSETS = {
        /* kP */    0.000095,
        /* kI */    0.0000009325,
        /* kD */    0.0,
        /* kF */    0.0,    //Feed forward gain constant
        /* I-Zne */ 0.0     //The range of error for kI to take affect (like a reverse deadband)
    };

    public static final double[][] SPIN_PID_CONFIG = {
        /* kP */    {1.91, 0.94, 0.98, 2.41},
        /* kI */    {0.0, 0.0, 0.0, 0.0},
        /* kD */    {70.6, 300.0, 300.0, 152.0},
        /* kF */    {0.0, 0.0, 0.0, 0.0}    //Feed forward gain constant
    };

    // public static final int[] OFFSETS = {2768, 2432, 2494, 3016};
    public static final int[] OFFSETS = {4846, 6575, 2456, 7081};

    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final double DRIVE_RAMP_RATE = 0.2;
    public static final double DRIVE_SPEED_MAX_EMPIRICAL_FPS = 13.79;
}
