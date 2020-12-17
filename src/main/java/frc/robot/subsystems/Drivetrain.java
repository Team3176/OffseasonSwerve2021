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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.DrivetrainConstants;

// import frc.robot.Controller;
// import frc.robot.VisionClient;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance = new Drivetrain();

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final SwervePod[] pods = new SwervePod[4];

  private double strafeCommand;
  private double forwardCommand;
  private double spinCommand;

  public Drivetrain() {
    gyro.reset();
    for(int i = 0; i < 4; i++) {
      pods[i] = new SwervePod(i);
    }
    strafeCommand = 0.0;
    forwardCommand = Math.pow(10, -15);
    spinCommand = 0.0;
  }
  
  public static Drivetrain getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
  }

  public void drive(double rotX, double transMag, double transAngle) {
    //Wheels are numbered according to the unit circle
    

    double a = strafeCommand - (spinCommand * getRadius("A"));
    double b = strafeCommand + (spinCommand * getRadius("B"))
    double c = forwardCommand - (spinCommand * getRadius("C"))
    double d = forwardCommand + (spinCommand * getRadius("D"))

    double wheel1Speed = Math.sqrt(Math.pow(b, 2), Math.pow(c, 2));
    double wheel2Speed = Math.sqrt(Math.pow(b, 2), Math.pow(d, 2));
    double wheel3Speed = Math.sqrt(Math.pow(a, 2), Math.pow(d, 2));
    double wheel4Speed = Math.sqrt(Math.pow(a, 2), Math.pow(c, 2));

    //In radians
    double wheel1Angle = atan2(b, c);
    double wheel2Angle = atan2(b, d);
    double wheel3Angle = atan2(a, d);
    double wheel4Angle = atan2(a, c);
    //Set thrust and spin for all wheels
  }

  private double getRadius(String letter) {
    if()
  }
}
