package frc.robot.constants;

public final class SwervePodConstants {

    private static final double WHEEL_DIAMETER = DrivetrainConstants.WHEEL_DIAMETER;  // in inches
    private static final double SPIN_GEAR_RATIO = 70.0 / 1.0; // Is the Versa gearbox btwn motor & encoder
    private static final double DRIVE_GEAR_RATIO = (54.0 / 14.0) * (48.0 / 30.0);  // 216/35?
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

    // DEFINE FPS2UPS (Conversion factor to change Feet-per-second to encoder-tics-per-second)
    // (12 in/sec) / (WHEEL_DIAMETER * PI) * 4096.0/10.0 * (1 / DRIVE_GEAR_REDUCTION)
    //          TODO:  The 4096 is # of tics per rev.  But where does the # "10" come from?
    
    // Distance_traveled_in_one_revolution = Wheel Circumfrance 
    // WHEEL_CIRCUMFERENCE = Math.PI * SwervePodConstants.WHEEL_DIAMETER
    // Speed_in_FT_per_Second = WHEEL_CIRCUMFERENCE * Number_of_Revolutions_per_1_second
    // 


    public static final double SPIN_ENCODER_UNITS_PER_REVOLUTION = 4096;
    public static final double DRIVE_ENCODER_UNITS_PER_REVOLUTION = 2048;

    public static final int TALON_PID_SLOT_ID = 0;  // TODO: Is this for Spin, Drive, or Both?

    public static final int TALON_PID_LOOP_ID = 0;  // TODO: Is this for Spin, Drive, or Both?

    public static final int TALON_TIMEOUT_MS = 0;   // TODO: Is this for Spin, Drive, or Both?
        
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
        /* kP */    {1.0, 2.0, 0.9, 0.1},
        /* kI */    {0.0, 0.0, 0.0, 0.0},
        /* kD */    {25.0, 50.0, 500.0, 100.0},
        /* kF */    {0.0, 0.0, 0.0, 0.0}    // Feed forward gain constant
    };


    /* OFFSETS: Corresponds to selftest output from CTRE Phoenix tool.
    *  Look for the absolute position encoder value.  Should say something 
    like:  "Pulsewidth/MagEnc(abs)"
    * Used solely for the Spin Encoder.
    */
    // public static final int[] OFFSETS = {4846, 6575, 2456, 7081};
    public static final int[] SPIN_OFFSET = {-5538, 44, 3135, -2963842}; 

    public static final double DRIVE_SPEED_MAX_EMPIRICAL_FEET_PER_SECOND = 13.79;

}
