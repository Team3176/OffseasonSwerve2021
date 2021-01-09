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

    // Tuning values for Drivetrain
    public static final double[][] LEFT_PID = {{0,0,0}, {0,0,0}};
    public static final double[][] LEFT_FF = {{0,0,0}, {0.142, 2.47, 0.205}};
    public static final double[][] RIGHT_PID = {{0,0,0}, {0,0,0}};
    public static final double[][] RIGHT_FF = {{0,0,0}, {0.142, 2.47, 0.205}};

    // Encoder Conversion Ratios
    public static final double VELOCITY_RATIO = (7.1631/10000.0)*0.3048;
    public static final double POSITION_RATIO = (7.1631/100000.0)*0.3048;

    // Drivetrain dimensions for kinematics and odometry
    public static final double LENGTH = 30.5; // Inches
    public static final double WIDTH = 29.5; // Inches
    public static final double WHEEL_DIAMETER = 3.25; // Inches
    public static final double MAX_WHEEL_SPEED = 13.5; // ft/s
    public static final double MAX_VEL = 6000; // Unknown units
    public static final double MAX_ACCEL = 5; // Unknown units - likely ft/s*s
    public static final double MAX_ROT_SPEED = 5.0; // rad/s
    public static final int ENCODER_UNITS = 4096; // tics

    // Speed limits
    /*
    public static final double MAX_LINEAR_SPEED = 4;
    public static final double MAX_LINEAR_ACCEL = 1;
    public static final double MAX_ROT_SPEED = 2.0 * Math.PI;
    public static final double OPEN_LOOP_RAMP_RATE = 0.5;
    */

    // PID values for DriveToSetpointPID
    public static final double DRIVE_TO_SET_POINT_ANGLE_P = 0.0;
    public static final double DRIVE_TO_SET_POINT_ANGLE_I = 0.0;
    public static final double DRIVE_TO_SET_POINT_ANGLE_D = 0.0;
    public static final double DRIVE_TO_SET_POINT_DISTANCE_P = 0.0;
    public static final double DRIVE_TO_SET_POINT_DISTANCE_I = 0.0;
    public static final double DRIVE_TO_SET_POINT_DISTANCE_D = 0.0;
}
