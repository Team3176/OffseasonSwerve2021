package frc.robot.subsystems;

//TODO: Recognize the red dependecys because seeing red is annoying
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  private coordType currentCoordType;
  private driveMode currentDriveMode;

  private boolean autonVision;

  private double lastGyroClock;

  public TalonFX[] driveControllers = {new TalonFX(DrivetrainConstants.DRIVE_ONE_CID),
                                            new TalonFX(DrivetrainConstants.DRIVE_TWO_CID),
                                            new TalonFX(DrivetrainConstants.DRIVE_THREE_CID),
                                            new TalonFX(DrivetrainConstants.DRIVE_FOUR_CID)};

  public TalonSRX[] spinControllers = {new TalonSRX(DrivetrainConstants.STEER_ONE_CID),
                                        new TalonSRX(DrivetrainConstants.STEER_TWO_CID),
                                        new TalonSRX(DrivetrainConstants.STEER_THREE_CID),
                                        new TalonSRX(DrivetrainConstants.STEER_FOUR_CID)};

  private double length;   // robot's wheelbase 
  private double width;    // robot's trackwidth
  private double k_etherRadius;  // radius used in A,B,C,D component calc's of ether decomposition

  private double maxSpeed_InchesPerSec;
  private double maxVel;
  private double maxRotation;
  private double maxAccel;

  private double relMaxSpeed;
  private double currentAngle;
  private double lastAngle;

  private double startTime = 0;
  private double currentTIme = 0;

  private boolean isVisionDriving;

  private double forwardCommand;
  private double strafeCommand;
  private double spinCommand;
 
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

  private SwervePod podFR;
  private SwervePod podFL;
  private SwervePod podBL;
  private SwervePod podBR;

  private Drivetrain() {
    // Instantiate pods
    podFR = new SwervePod(0, driveControllers[0], spinControllers[0]);
    podFL = new SwervePod(1, driveControllers[1], spinControllers[1]);
    podBL = new SwervePod(2, driveControllers[2], spinControllers[2]);
    podBR = new SwervePod(3, driveControllers[3], spinControllers[3]);

    // Instantiate array list then add instantiated pods to list
    pods = new ArrayList<SwervePod>();
    pods.add(podFR);
    pods.add(podFL);
    pods.add(podBL);
    pods.add(podBR);

    //currentCoordType = coordType.FIELD_CENTRIC;
    currentCoordType = coordType.ROBOT_CENTRIC;

    autonVision = false;

    // Setting constants
    length = DrivetrainConstants.LENGTH;
    width = DrivetrainConstants.WIDTH;
    k_etherRadius = Math.sqrt(Math.pow(length,2) / Math.pow(width,2))/2; 

    maxSpeed_InchesPerSec = DrivetrainConstants.MAX_WHEEL_SPEED_INCHES_PER_SECOND;
    maxRotation = DrivetrainConstants.MAX_ROT_SPEED;
    maxAccel = DrivetrainConstants.MAX_ACCEL;

    // Instantiating the gyro
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
    updateAngle();
    SmartDashboard.putNumber("currentAngle", currentAngle);

    SmartDashboard.putNumber("forwardCommand", 0);
    SmartDashboard.putNumber("strafeCommand", 0);
    SmartDashboard.putNumber("spinCommand", 0);

    isVisionDriving = false;

    // TODO: We initialize to face forward but how do we make this into a command?
    // Maybe we say drive with the below parameters, but where?
    /*
    // Start wheels in a forward facing direction */
    
    this.forwardCommand = Math.pow(10, -15); // Has to be positive to turn that direction?
    this.strafeCommand = 0.0;
    this.spinCommand = 0.0;
    
  }
  
  // Prevents more than one instance of drivetrian
  public static Drivetrain getInstance() { return instance; }
/** 
  public void drive(double forwardCommand, double strafeCommand, double spinCommand, int uselessVariable) {
    double smallNum = Math.pow(10, -15);
    //spinCommand = (spinCommand - (-1))/(1 - (-1));  //rescales spinCommand to a 0..1 range
    double angle = (spinCommand * Math.PI) + Math.PI;  // <- diff coord system than -1..1 = 0..2Pi
                                                       // This coord system is 0..1 = Pi..2Pi, & 
                                                       //                      0..-1 = Pi..-2PI
                                                       //                      right?
                                                       //             Fixed by new rescaling at line 140?
    pods.get(0).set(smallNum, angle);
  }
  */


  /**
   * 
   * @param forwardCommand range of {-1,1}
   * @param strafeCommand range of {-1, 1}
   * @param spinCommand range of {-1, 1}
   */
  public void drive(double forwardCommand, double strafeCommand, double spinCommand) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;

    this.forwardCommand = SmartDashboard.getNumber("forwardCommand", 0);
    this.strafeCommand = SmartDashboard.getNumber("strafeCommand", 0);
    this.spinCommand = SmartDashboard.getNumber("spinCommand", 0);

    SmartDashboard.putNumber("drive()InputForwardCommand", forwardCommand);
    SmartDashboard.putNumber("drive()InputStrafeCommand", strafeCommand);
    SmartDashboard.putNumber("drive()InputSpinCommand", spinCommand);

    

    // TODO: Make the gyro reset if a certain button is pushed
    updateAngle();
    SmartDashboard.putNumber("Drive updated currentAngle Degrees", (currentAngle * 180/Math.PI));


    if(currentDriveMode != driveMode.TURBO) {
      this.forwardCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
      this.strafeCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
      this.spinCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
    }

    if(currentCoordType == coordType.FIELD_CENTRIC) {
      final double temp = (this.forwardCommand * Math.sin(currentAngle) + this.strafeCommand * Math.cos(currentAngle));
      this.strafeCommand = (this.forwardCommand * Math.cos(currentAngle) + this.strafeCommand * Math.sin(currentAngle));
      this.forwardCommand = temp;
    }
    // TODO: Find out why we multiply by 0.75
    if(currentCoordType == coordType.ROBOT_CENTRIC) {
      this.strafeCommand *= 1; //0.75;
      this.forwardCommand *= 1; //0.75;
      this.spinCommand *= 1; //0.75;
    }
    if(currentCoordType == coordType.BACK_ROBOT_CENTRIC) {
      this.strafeCommand *= -1;
      this.forwardCommand *= -1;
    }
    SmartDashboard.putNumber("this.forwardComDriveTrain.drive", this.forwardCommand);
    SmartDashboard.putNumber("this.strafeComDriveTrain.drive", this.strafeCommand);
    SmartDashboard.putNumber("this.spinComDriveTrain.drive", this.spinCommand);
    calculateNSetPodPositions(this.forwardCommand, this.strafeCommand, this.spinCommand);    
  }

  /**
   * 
   * @param forwardCommand range of {-1,1} coming from translation stick Y-Axis
   * @param strafeCommand range of {-1,1} coming from translation stick X-Axis
   * @param spinCommand range of {}
   */
  private void calculateNSetPodPositions(double forwardCommand, double strafeCommand, double spinCommand) {
    
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand; 
    this.spinCommand = spinCommand; 

    if(currentDriveMode != driveMode.DEFENSE) {
      // Create arrays for the speed and angle of each pod
      double[] podDrive = new double[4];
      double[] podSpin = new double[4];

      /*
      // ###########################################################
      // BEGIN:   Ether Eqns -- Ether's official derivation
      //          +Y :=  axis of chassis forward movement
      //          +X :=  axis of chassis strafe to starboard/right
      // ###########################################################
      double a = strafeCommand --spinCommand * length/2; 
      double b = strafeCommand + spinCommand * length/2;
      double c = forwardCommand - spinCommand * width/2;  
      double d = forwardCommand + spinCommand * width/2;
      
      // Calculate speed (podDrive[idx]) and angle (podSpin[idx]) of each pod
      podDrive[0] = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));
      podSpin[0] = Math.atan2(b, c);
      
      podDrive[1] = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
      podSpin[1] = Math.atan2(b, d);

      podDrive[2] = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));
      podSpin[2] = Math.atan2(a, d);

      podDrive[3] = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));
      podSpin[3] = Math.atan2(a, c);
      // ###########################################################
      // /END     Ether Eqns -- Ether's official derivation
      // ###########################################################
      */



      // ###########################################################
      // BEGIN:   Ether Eqns -- JonH derivation 2021-02-15
      //          +X :=  axis of chassis forward movement ... we think
      //          -Y :=  axis of chassis strafe to starboard/right ... we think
      // ###########################################################
      double a = this.forwardCommand - this.spinCommand * width/2;
      double b = this.forwardCommand + this.spinCommand * width/2;
      double c = this.strafeCommand - this.spinCommand * length/2;
      double d = this.strafeCommand + this.spinCommand * length/2;

      // Calculate speed and angle of each pod
      // TODO: Verify order of atan2 parameters. atan2(y,x) is formal java def, 
      //        but past implementations and ether use atan2(x,y).
      podDrive[0] = Math.sqrt(Math.pow(b,2) + Math.pow(d,2));
      podSpin[0] = Math.atan2(d, b);

      podDrive[1] = Math.sqrt(Math.pow(b,2) + Math.pow(c,2));
      podSpin[1] = Math.atan2(c, b);
      
      podDrive[2] = Math.sqrt(Math.pow(a,2) + Math.pow(c,2));
      podSpin[2] = Math.atan2(c, a);
      
      podDrive[3] = Math.sqrt(Math.pow(a,2) + Math.pow(d,2));
      podSpin[3] = Math.atan2(d, a);
      // ###########################################################
      // END:   Ether Eqns -- JonH derivation 2021-02-15
      // ###########################################################

      SmartDashboard.putNumber("a", a);
      SmartDashboard.putNumber("b", b);
      SmartDashboard.putNumber("c", c);
      SmartDashboard.putNumber("d", d);
      for (int idx = 0; idx < 4; idx++) {
        SmartDashboard.putNumber("preScale P" + (idx + 1) + " podDrive", podDrive[idx]);
      }

      // Find the highest pod speed then normalize if a pod is exceeding our max speed
      relMaxSpeed = Math.max(Math.max(podDrive[0], podDrive[1]), Math.max(podDrive[2], podDrive[3]));
      if(relMaxSpeed > maxSpeed_InchesPerSec) {
        for(int idx = 0; idx < pods.size(); idx++) {
          podDrive[idx] /= relMaxSpeed / maxSpeed_InchesPerSec;
        }
      }

      // Set calculated drive and spins to each pod
      for(int idx = 0; idx < pods.size(); idx++) {
        pods.get(idx).set(podDrive[idx], podSpin[idx]);   
      }

    } else { // Enter defenseive position
      double smallNum = Math.pow(10, -15);
      pods.get(0).set(smallNum, -1.0 * Math.PI / 4.0);
      pods.get(1).set(smallNum, 1.0 * Math.PI / 4.0);
      pods.get(2).set(smallNum, 3.0 * Math.PI / 4.0);
      pods.get(3).set(smallNum, -3.0 * Math.PI / 4.0);
    }
  }

  private void updateAngle() {
    // -pi to pi; 0 = straight
    currentAngle = ((((gyro.getAngle()) * Math.PI/180.0)) % (2*Math.PI));
    //currentAngle value is in radians0
  }

  private double getRadius(String component) {
    // Omitted if driveStatements where we pivoted around a pod
    // This'll be orbit and dosado in the future
    if(false /* orbiting || dosadoing */) {
      // Do special things to components based on radius and more
    } else {
      if(component.equals("A") || component.equals("B")) { return length / 2 ; }
      else { return width / 2; }  //TODO: place to check for forward vs back pods working vs not working
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
