package frc.robot.constants;

public final class SwervePodConstants {

    private static final double WHEEL_DIAMETER = 3.25; //Inches
    private static final double SPIN_GEAR_RATIO = 70.0 / 1.0;
    private static final double DRIVE_GEAR_RATIO = (54.0 / 14.0) * (48.0 / 30.0);
    // private static final double 
    // SPIN
    // motor spins 140 rev/min
    // after 70 to 1 --> 2 rev/min
    // encoder reads 4096 * 2
    // wheel spins 2 rev/min
    
    // DRIVE
    // 54 / 14
    // 48 / 30
    // So 6.17... motor revolutions means 1 wheel rotation

    public static final double SPIN_ENCODER_UNITS_PER_REVOLUTION = 4096;
    public static final double DRIVE_ENCODER_UNITS_PER_REVOLUTION = 2048;

    public static final int TALON_PID_SLOT_ID = 0;

    public static final int TALON_PID_LOOP_ID = 0; 

    public static final int TALON_TIMEOUT_MS = 0;
        
    public static final double[][] DRIVE_PID = {
        /* kP */    {0.15, 0.15, 0.15, 0.15},
        /* kI */    {0.0, 0.0, 0.0, 0.0},
        /* kD */    {0.0, 0.0, 0.0, 0.0},
        /* kF */    {0.0, 0.0, 0.0, 0.0},    // Feed forward gain constant
        /* I-Zne */ {0.0, 0.0, 0.0, 0.0}     // The range of error for kI to take affect (like a reverse deadband)
    };
    // BR P: 2.41, I: 0.0, D: 152.0, F: 0.0
    public static final double[][] SPIN_PID = {
        //           FR    FL    BL     BR
        /* kP */    {1.0, 2.0, 0.9, 1.0},
        /* kI */    {0.0, 0.0, 0.0, 0.0},
        /* kD */    {25.0, 50.0, 500.0, 100.0},
        /* kF */    {0.0, 0.0, 0.0, 0.0}    // Feed forward gain constant
    };
    // public static final int[] OFFSETS = {4846, 6575, 2456, 7081};
    public static final int[] OFFSETS = {-288119, -983005, -177113, 55045};

    public static final double DRIVE_SPEED_MAX_EMPIRICAL_FPS = 13.79;

}
