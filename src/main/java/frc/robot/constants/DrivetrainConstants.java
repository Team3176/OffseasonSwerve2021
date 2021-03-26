package frc.robot.constants;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.constants.MasterConstants;

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
    public static final double LENGTH_2019 = 30.5; // Inches
    public static final double LENGTH_2021 = 58.1 * (1.0/2.54); // Inches but measured in cm

    public static final double WIDTH_2019 = 29.5; // Inches
    public static final double WIDTH_2021 = 64.0 * (1.0/2.54); // Inches but measured in cm

    public static double LENGTH = (MasterConstants.is2021Bot) ?  LENGTH_2021 : LENGTH_2019;
    public static double WIDTH = (MasterConstants.is2021Bot) ?  WIDTH_2021 : WIDTH_2019;
	public static double DRIVE_ENCODER_UNITS_PER_REVOLUTION;

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
    
    public static final double MAX_ROT_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 1; // rad/s*s

    public static final double NON_TURBO_PERCENT_OUT_CAP = 0.5;


    public static final double LENGTH_CENTER_TOCENTER = 23.5;
    public static final double WIDTH_CENTER_TCENTER = 23.5;
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = 
    new SwerveDriveKinematics(
    new Translation2d(Units.inchesToMeters(LENGTH_CENTER_TOCENTER) / 2, Units.inchesToMeters(WIDTH_CENTER_TCENTER) / 2),
    new Translation2d(-Units.inchesToMeters(LENGTH_CENTER_TOCENTER) / 2, Units.inchesToMeters(WIDTH_CENTER_TCENTER) / 2),
    new Translation2d(-Units.inchesToMeters(LENGTH_CENTER_TOCENTER) / 2, -Units.inchesToMeters(WIDTH_CENTER_TCENTER) / 2),
    new Translation2d(Units.inchesToMeters(LENGTH_CENTER_TOCENTER) / 2, -Units.inchesToMeters(WIDTH_CENTER_TCENTER) / 2));

    public static final double P_THETA_CONTROLLER = 1;

    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
    new TrapezoidProfile.Constraints(
        //MAX_ROT_SPEED_RADIANS_PER_SECOND, MAX_ROT_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
        5, 1);

    public static final double P_X_Controller = 1;
    public static final double P_Y_Controller = 1;
    public static final double P_Theta_Controller = 1;
    
    public static final double DEGREES_PER_SECOND_TO_METERS_PER_SECOND_OF_WHEEL = (Units.feetToMeters(WHEEL_DIAMETER_FEET)*Math.PI)/360;

    public static final double P_MODULE_DRIVE_CONTROLLER = 1;
    public static final double P_MODULE_TURNING_CONTROLLER = 1;
    public static final double MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
    public static final double MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2* Math.PI;
}
