package frc.robot.constants;

public class DrivetrainConstants {
    // IDs for Drivetrain motors and solenoids

    public static final int LEFT_MASTER_ID =  2;
    public static final int LEFT_FOLLOWER_ID = 22 ;
    public static final int RIGHT_MASTER_ID =  1;
    public static final int RIGHT_FOLLOWER_ID = 11;

    public static final int RIGHT_SHIFTER_ID_1 = 1;
    public static final int RIGHT_SHIFTER_ID_2 = 6;
    public static final int LEFT_SHIFTER_ID_1 = 7;
    public static final int LEFT_SHIFTER_ID_2 = 0;

    // Tuning values for Drivetrain
    public static final double[][] LEFT_PID = {{0,0,0}, {0,0,0}};
    public static final double[][] LEFT_FF = {{0,0,0}, {0.142, 2.47, 0.205}};
    public static final double[][] RIGHT_PID = {{0,0,0}, {0,0,0}};
    public static final double[][] RIGHT_FF = {{0,0,0}, {0.142, 2.47, 0.205}};

    // Encoder Conversion Ratios
    public static final double VELOCITY_RATIO = (7.1631/10000.0)*0.3048;
    public static final double POSITION_RATIO = (7.1631/100000.0)*0.3048;

    // Drivetrain width for kinematics and odometry
    public static final double TRACK_WIDTH = 0.6096;

    // Speed limits
    public static final double MAX_LINEAR_SPEED = 4;
    public static final double MAX_LINEAR_ACCEL = 1;
    public static final double MAX_ROT_SPEED = 2.0 * Math.PI;
    public static final double OPEN_LOOP_RAMP_RATE = 0.5;

    // PID values for DriveToSetpointPID
    public static final double DRIVETOSETPOINT_ANGLE_P = 0.0;
    public static final double DRIVETOSETPOINT_ANGLE_I = 0.0;
    public static final double DRIVETOSETPOINT_ANGLE_D = 0.0;
    public static final double DRIVETOSETPOINT_DISTANCE_P = 0.0;
    public static final double DRIVETOSETPOINT_DISTANCE_I = 0.0;
    public static final double DRIVETOSETPOINT_DISTANCE_D = 0.0;
}
