package frc.robot.constants;

public class DrivetrainConstants {
    // IDs for Drivetrain motors and solenoids

    //CAN IDs
    public static final int DRIVE_ONE_CID = 1;
    public static final int DRIVE_TWO_CID = 2;
    public static final int DRIVE_THREE_CID = 3;
    public static final int DRIVE_FOUR_CID = 4;

    //CAN IDs
    public static final int STEER_ONE_CID = 11;
    public static final int STEER_TWO_CID = 22;
    public static final int STEER_THREE_CID = 33;
    public static final int STEER_FOUR_CID = 44;

    // Drivetrain dimensions for kinematics and odometry
    public static final double LENGTH = 58.1 * (1.0/2.54); // Inches but actually 58.1 cm
    public static final double WIDTH = 64.0 * (1.0/2.54); // Inches but actually 64 cm

    public static final double WHEEL_DIAMETER_INCHES = 3.25; // Inches
    public static final double WHEEL_DIAMETER_FEET = WHEEL_DIAMETER_INCHES / 12.0 ; // Inches
    public static final double WHEEL_DIAMETER = WHEEL_DIAMETER_INCHES;

    public static final double MAX_WHEEL_SPEED_INCHES_PER_SECOND = 165.48; // inches/s
    public static final double MAX_WHEEL_SPEED_FEET_PER_SECOND = MAX_WHEEL_SPEED_INCHES_PER_SECOND / 12.0 ; // 13.5 ft/s
    public static final double MAX_WHEEL_SPEED = MAX_WHEEL_SPEED_INCHES_PER_SECOND;

    public static final double MAX_ACCEL_INCHES_PER_SECOND = 60.0; // inches/s*s
    public static final double MAX_ACCEL_FEET_PER_SECOND = MAX_ACCEL_INCHES_PER_SECOND / 12.0; // Unknown units - likely ft/s*s
    public static final double MAX_ACCEL = MAX_ACCEL_INCHES_PER_SECOND;

    public static final double MAX_ROT_SPEED_RADIANS_PER_SECOND = 5.0; // rad/s
    public static final double MAX_ROT_SPEED = 5.0; // rad/s


    public static final double NON_TURBO_PERCENT_OUT_CAP = 0.5;
}