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
    public static final double LENGTH = 30.5; // Inches
    public static final double WIDTH = 29.5; // Inches
    public static final double WHEEL_DIAMETER = 3.25; // Inches
    public static final double MAX_ROBOT_SPEED = 13.79; // ft/s
}