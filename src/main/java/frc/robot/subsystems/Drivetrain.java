package frc.robot.subsystems;

//TODO: Recognize the red dependecys because seeing red is annoying
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.DrivetrainConstants;
import frc.robot.Controller;
// import frc.robot.Controller;
// import frc.robot.VisionClient;

import java.util.ArrayList;
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

  private coordType currentCoordType;
  private driveMode currentDriveMode;

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

  private double relMaxSpeed;
  private double currentAngle;
  private double lastAngle;

  private double startTime = 0;
  private double currentTIme = 0;

  private boolean isVisionDriving;

  public enum driveMode {
    DEFENSE,
    DRIVE,
    TURBO,
    VISION
  }

  public enum coordType {
    BACK_ROBOT_CENTRIC,
    FIELD_CENTRIC,
    ROBOT_CENTRIC
  }

  private Drivetrain() {
    // Instantiate pods
    podFR = new SwervePod(0, driveControllers[0], spinControllers[0]);
    podFL = new SwervePod(1, driveControllers[1], spinControllers[1]);
    podBR = new SwervePod(2, driveControllers[2], spinControllers[2]);
    podBL = new SwervePod(3, driveControllers[3], spinControllers[3]);

    currentCoordType = coordType.FIELD_CENTRIC;

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

    // TODO: We initialize to face forward but how do we make this into a command?
    // Maybe we say drive with the below parameters, but where?
    /*
    // Start wheels in a forward facing direction
    forwardCommand = Math.pow(10, -15); // Has to be positive to turn that direction?
    strafeCommand = 0.0;
    spinCommand = 0.0;
    */
  }
  
  // Prevents more than one instance of drivetrian
  public static Drivetrain getInstance() { return instance; }

  public void drive(double forwardCommand, double strafeCommand, double spinCommand) {
    // TODO: Make the gyro reset if a certain button is pushed
    updateAngle();

    if(currentDriveMode != driveMode.TURBO) {
      forwardCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
      strafeCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
      spinCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
    }

    if(currentCoordType == coordType.FIELD_CENTRIC) {
      final double temp = forwardCommand * Math.sin(currentAngle) + strafeCommand * Math.cos(currentAngle);
      strafeCommand = (-forwardCommand * Math.cos(currentAngle) + strafeCommand * Math.sin(currentAngle));
      forwardCommand = temp;
    }
    // TODO: Find out why we multiply by 0.75
    if(currentCoordType == coordType.ROBOT_CENTRIC) {
      strafeCommand *= 0.75;
      forwardCommand *= 0.75;
      spinCommand *= 0.75;
    }
    if(currentCoordType == coordType.BACK_ROBOT_CENTRIC) {
      strafeCommand *= -1;
      forwardCommand *= -1;
    }

    calculateNSetPodPositions(forwardCommand, strafeCommand, spinCommand);
  }

  private void calculateNSetPodPositions(double forwardCommand, double strafeCommand, double spinCommand) {
    
    if(currentDriveMode != driveMode.DEFENSE) {
      // Create arrays for the speed and angle of each pod
      double[] podDrive = new double[4];
      double[] podSpin = new double[4];

      double a = strafeCommand + spinCommand * getRadius("A");
      double b = strafeCommand - spinCommand * getRadius("B");
      double c = forwardCommand - spinCommand * getRadius("C");
      double d = forwardCommand + spinCommand * getRadius("D");

      // Calculate speed and angle of each pod
      podDrive[0] = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));
      podSpin[0] = Math.atan2(b, c);

      podDrive[1] = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
      podSpin[1] = Math.atan2(b, d);

      podDrive[2] = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));
      podSpin[2] = Math.atan2(a, c);

      podDrive[3] = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));
      podSpin[3] = Math.atan2(a, c);

      // Find the highest pod speed then normalize if a pod is exceeding our max speed
      relMaxSpeed = Math.max(Math.max(podDrive[0], podDrive[1]), Math.max(podDrive[2], podDrive[3]));
      if(relMaxSpeed > maxSpeed) {
        for(int i = 0; i < pods.size(); i++) {
          podDrive[i] /= (relMaxSpeed / maxSpeed);
        }
      }

      // Set calculated drive and spins to each pod
      for(int i = 0; i < pods.size(); i++) {
        pods.get(i).set(podDrive[i], podSpin[i]);
      }

    } else { // Enter defenseive position
      pods.get(0).set(0.0, -1.0 * Math.PI * 4.0);
      pods.get(1).set(0.0, 1.0 * Math.PI * 4.0);
      pods.get(2).set(0.0, 3.0 * Math.PI * 4.0);
      pods.get(3).set(0.0, -3.0 * Math.PI * 4.0);
    }
  }

  private void updateAngle() {
    // -pi to pi; 0 = straight
    currentAngle = ((((gyro.getAngle() + 90) * Math.PI/180.0)) % (2*Math.PI));
  }

  private double getRadius(String component) {
    // Omitted if driveStatements where we pivoted around a pod
    // This'll be orbit and dosado in the future
    if(false /* orbiting || dosadoing */) {
      // Do special things to components based on radius and more
    } else {
      if(component.equals("A") || component.equals("B")) { return length / 2.0 ; }
      else { return width / 2.0; }
    }
    return 0.0;
  }

  public void setDriveMode(driveMode wantedDriveMode) {
    currentDriveMode = wantedDriveMode;
  }

  public void setCoordType(coordType wantedType) {
    currentCoordType = wantedType;
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
