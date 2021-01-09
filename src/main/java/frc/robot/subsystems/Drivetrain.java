package frc.robot.subsystems;

//TODO: Clean these later
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import java.util.ArrayList;
import frc.robot.constants.DrivetrainConstants;
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
import frc.robot.Controller;
// import frc.robot.Controller;
// import frc.robot.VisionClient;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance = new Drivetrain();

  private Controller controller = Controller.getInstance();
  private PowerDistributionPanel PDP = new PowerDistributionPanel(0);
  private AHRS gyro;

  private ArrayList<SwervePod> pods;

  private SwervePod podFR;
  private SwervePod podFL;
  private SwervePod podBL;
  private SwervePod podBR;

  private coordType mCoordType;
  private inputType mInputType;

  private boolean autonVision;

  private double lastGyroClock;

  public CANSparkMax[] driveControllers = {new CANSparkMax(DrivetrainConstants.DRIVE_ONE_CID, MotorType.kBrushless),
                                           new CANSparkMax(DrivetrainConstants.DRIVE_TWO_CID, MotorType.kBrushless),
                                           new CANSparkMax(DrivetrainConstants.DRIVE_THREE_CID, MotorType.kBrushless),
                                           new CANSparkMax(DrivetrainConstants.DRIVE_FOUR_CID, MotorType.kBrushless)};

  public TalonSRX[] spinControllers = {new TalonSRX(DrivetrainConstants.STEER_ONE_CID),
                                        new TalonSRX(DrivetrainConstants.STEER_TWO_CID),
                                        new TalonSRX(DrivetrainConstants.STEER_THREE_CID),
                                        new TalonSRX(DrivetrainConstants.STEER_FOUR_CID)};

  private double length;
  private double width;

  private double maxSpeed;
  private double maxVel;
  private double maxRotation;
  private double maxAccel;

  private double relMaxSPeed;
  private double currentAngle;
  private double lastAngle;

  private double forwardCommand;
  private double strafeCommand;
  private double spinCommand;

  private double startTime = 0;
  private double currentTIme = 0;

  private boolean isVisionDriving;

  public enum state {
    NEUTRAL,
    DRIVE,
    AUTON
  }

  public enum coordType {
    ROBOT_CENTRIC,
    FIELD_CENTRIC,
    BACK_ROBOT_CENTRIC
  }

  public enum inputType {
    PERCENT_POWER,
    VELOCITY
  }

  private state currentState;
  private state wantedState;
  private state lastState;

  private Drivetrain() {
    // Instantiate pods
    podFR = new SwervePod(0, driveControllers[0], spinControllers[0]);
    podFL = new SwervePod(1, driveControllers[1], spinControllers[1]);
    podBR = new SwervePod(2, driveControllers[2], spinControllers[2]);
    podBL = new SwervePod(3, driveControllers[3], spinControllers[3]);

    mCoordType = coordType.FIELD_CENTRIC;
    mInputType = inputType.PERCENT_POWER;

    autonVision = false;

    // Instantiate array list then add instantiated pods to list
    pods = new ArrayList<SwervePod>();
    pods.add(podFR);
    pods.add(podFL);
    pods.add(podBL);
    pods.add(podBR);

    // Setting constants
    length = DrivetrainConstants.LENGTH;
    width = DrivetrainConstants.WIDTH;

    maxSpeed = DrivetrainConstants.MAX_WHEEL_SPEED;
    maxRotation = DrivetrainConstants.MAX_ROT_SPEED;
    maxVel = DrivetrainConstants.MAX_VEL; // Not used - don't know what it is
    maxAccel = DrivetrainConstants.MAX_ACCEL;

    // Instantiating the gyro
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
    updateAngle();

    isVisionDriving = false;

    // Start wheels in a forward facing direction
    forwardCommand = Math.pow(10, -15); // Not sure why this is >0
    strafeCommand = 0.0;
    spinCommand = 0.0;
  }
  
  // Prevents more than one instance of drivetrian
  public static Drivetrain getInstance() {
    return instance;
  }

  private void drive() {
    if(mCoordType == coordType.FIELD_CENTRIC) {
      final double temp = forwardCommand * Math.sin(currentAngle) + strafeCommand * Math.cos(currentAngle);
      strafeCommand = (-forwardCommand * Math.cos(currentAngle) + strafeCommand * Math.sin(currentAngle));
      forwardCommand = temp;
    }
    if (mCoordType == coordType.BACK_ROBOT_CENTRIC) {
      strafeCommand *= -1;
      forwardCommand *= -1;
    }
    if(mCoordType == coordType.ROBOT_CENTRIC) {
      strafeCommand *= 0.75;
      forwardCommand *= 0.75;
      spinCommand *= 0.75;
    }
    if(mInputType == inputType.PERCENT_POWER) {
      strafeCommand *= maxSpeed;
      forwardCommand *= maxSpeed;
      spinCommand *= maxRotation;
    }

    
  }

  @Override
  public void periodic() {
  }

  private void updateAngle() {
    // -pi to pi; 0 = straight
    currentAngle = ((((gyro.getAngle() + 90) * Math.PI/180.0)) % (2*Math.PI));
  }

  /*
  public void drive(double drivePercent, double spinPercent) {
    SmartDashboard.putBoolean("Are we calling drive", true);
    pod1.velocityPIDDriveNSpin(drivePercent, spinPercent);
    // pod1.percentFFDriveNSpin(drivePercent, spinPercent);
    if(drivePercent > 1.4) {
      pod.velocityPIDDriveNSpin(5.0, 0.0);
    } else {
      pod.velocityPIDDriveNSpin(0.0, 0.0);
    }
  }
  */

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
