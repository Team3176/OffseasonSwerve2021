package frc.robot.constants;

public final class SwervePodConstants {

    private static final double WHEEL_DIAMETER = 3.25; //Inches
    private static final double SPIN_GEAR_RATIO = 70.0 / 1.0;
    private static final double DRIVE_GEAR_RATIO = (54.0 / 14.0) * (48.0 / 30.0);

    public static final double SPIN_ENCODER_UNITS_PER_REVOLUTION = 4096;
    public static final double DRIVE_ENCODER_UNITS_PER_REVOLUTION = 2048;

    public static final int TALON_PID_SLOT_ID = 0;
    public static final int TALON_PID_LOOP_ID = 0; 
    public static final int TALON_TIMEOUT_MS = 0;
        
    public static final double[][] DRIVE_PID = {
        /* kP */    {0.15, 0.15, 0.15, 0.15},
        /* kI */    {0.0, 0.0, 0.0, 0.0},
        /* kD */    {0.0, 0.0, 0.0, 0.0},
        /* kF */    {0.0, 0.0, 0.0, 0.0},    
        /* I-Zne */ {0.0, 0.0, 0.0, 0.0}    
    };

    public static final double[][] SPIN_PID = {
        //           FR    FL    BL     BR
        /* kP */    {1.0, 2.0, 0.9, 0.1},
        /* kI */    {0.0, 0.0, 0.0, 0.0},
        /* kD */    {25.0, 50.0, 500.0, 100.0},
        /* kF */    {0.0, 0.0, 0.0, 0.0}   
    };

    public static final int[] OFFSETS = {-5538, 44, 3135, -2963842};

    public static final double DRIVE_SPEED_MAX_EMPIRICAL_FPS = 13.79;

}
