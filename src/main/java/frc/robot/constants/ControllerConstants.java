package frc.robot.constants;

public class ControllerConstants {
    // ID values for joysticks and buttons
    public static final int DRIVE_STICK_ID = 0; //Left side
    public static final int SPIN_STICK_ID = 1;  //Right side
    public static final int OPERATOR_ID = 2;

    // Multipliers and other constants for Controller logic
    public static final double DEADBAND = 0.1;
    public static final double SLOW_SPEED_MULT = 0.5;
    public static final double SLOW_ROT_MULT = 0.5;
    public static final double TEMP_JOYSTICK_MIN_MAX = 2.827433;
}
