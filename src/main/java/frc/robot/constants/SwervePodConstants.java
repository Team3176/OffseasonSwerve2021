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

    /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 0;
	
	/* Choose so that Talon does not report sensor out of phase */
	public static boolean kSensorPhase = false;

	/**
	 * Choose based on what direction you want to be positive,
	 * this does not affect motor invert. 
	 */
    public static boolean kMotorInverted = false;



    public static final double SPIN_ENCODER_UNITS_PER_REVOLUTION = 4096;
    public static final int TALON_SPIN_PID_SLOT_ID = 0; 
    public static final int TALON_SPIN_PID_LOOP_ID = 0; 
    public static final int TALON_SPIN_PID_TIMEOUT_MS = 0;  

    public static final double DRIVE_ENCODER_UNITS_PER_REVOLUTION = 2048;
    public static final int TALON_DRIVE_PID_SLOT_ID = kSlotIdx; 
    public static final int TALON_DRIVE_PID_LOOP_ID = kPIDLoopIdx; 
    public static final int TALON_DRIVE_PID_TIMEOUT_MS = kTimeoutMs;  
    

    public static final double[][] DRIVE_PID = {
        /* kP */    {0.1, 0.11, 0.15, 0.23},
        // /* kI */    {0.001, 0.0015, 0.0015, 0.005}, // using these don't work
        /* kI */    {0.0, 0.0, 0.0, 0.0},
        /* kD */    {4.0, 7.0, 3.5, 6.0},
        /* kF */    {0.0435, 0.0446, 0.044, 0.0445},    // Feed forward gain constant
        /* I-Zne */ {0.0, 0.0, 0.0, 0.0}     // The range of error for kI to take affect (like a reverse deadband)
    };
    // BR P: 2.41, I: 0.0, D: 152.0, F: 0.0
    public static final double[][] SPIN_PID = {
        //           FR    FL    BL     BR
    //    /* kP */    {1.0, 2.0, 0.9, 0.1},
//        /* kP */    {1.0, 2.0, 0.9, 2.0},
                    {1.0, 1.0, 1.0, 1.0},
        /* kI */    {0.0, 0.0, 0.0, 0.0},
        ///* kD */    {25.0, 50.0, 500.0, 50.0},
                    {20.0, 20.0, 20.0, 20.0},
    //    /* kD */    {25.0, 50.0, 500.0, 100.0},
    /* kF */    {0.0, 0.0, 0.0, 0.0}    // Feed forward gain constant
    };


    /* OFFSETS: Corresponds to selftest output from CTRE Phoenix tool.
    *  Look for the absolute position encoder value.  Should say something 
    like:  "Pulsewidth/MagEnc(abs)"
    * Used solely for the Spin Encoder.
    */
    // public static final int[] OFFSETS = {4846, 6575, 2456, 7081};
    //public static final int[] SPIN_OFFSET = {-5538, 44, 3135, -2963842}; 
    //public static final int[] SPIN_OFFSET = {6821, 4143, -5071, 1765};
    //public static final int[] SPIN_OFFSET = {2689, 86, 31117, 3802830};
    //public static final int[] SPIN_OFFSET = {-1381, 63, -1019, 1740};
    // public static final int[] SPIN_OFFSET = {47762, 4189, 388129, -50897};
    // public static final int[] SPIN_OFFSET = {49844, 55384, 58418, 73442};
    public static final int[] SPIN_OFFSET_OLD_BOT = {-7492, -6034, -6649, -8475}; // 2019 Bot
    public static final int[] SPIN_OFFSET_NEW_BOT = {460, 4118, 6927, 2894}; // 2021 Bot
    public static final int[] SPIN_OFFSET = SPIN_OFFSET_OLD_BOT; // Must be manual changed
    public static final double DRIVE_SPEED_MAX_EMPIRICAL_FEET_PER_SECOND = 13.79;

}
