/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.DrivetrainConstants;
// import frc.robot.Controller;
// import frc.robot.VisionClient;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance = new Drivetrain();

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final SwervePod pod;


  //private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
  //    DrivetrainConstants.TRACK_WIDTH);

  //private final DifferentialDriveOdometry odometry;

  public Drivetrain() {
    gyro.reset();
    pod = new SwervePod();
    //odometry = new DifferentialDriveOdometry(getAngle());
  }
  
  public static Drivetrain getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
  }

  public void drive(double drivePercent, double spinPercent) {
    SmartDashboard.putBoolean("Are we calling drive", true);
    pod.percentControlDriveNSpin(drivePercent, spinPercent);
    /*
    if(drivePercent > 1.4) {
      pod.rpmControlDriveNSpin(5.0, 0.0);
    } else {
      pod.rpmControlDriveNSpin(0.0, 0.0);
    }
    */
  }

  /*
  public void setPose(Pose2d setPose) {
    odometry.resetPosition(setPose, getAngle());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public Pose2d getCurrentPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }
  */
}
