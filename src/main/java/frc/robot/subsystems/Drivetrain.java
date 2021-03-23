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


/* ##################################################################################
 *  BEGIN: Temporary Code for PIDController of rotation to stop drift in AutonCrude
 * ################################################################################## */
    import frc.robot.util.PIDLoop;
/* ##################################################################################
 *  END: Temporary Code for PIDController of rotation to stop drift in AutonCrude
 * ################################################################################## */
public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance = new Drivetrain();
  private Controller controller = Controller.getInstance();

  private PowerDistributionPanel PDP = new PowerDistributionPanel(0);
  private AHRS gyro;
  private double gyroOffset = 0;

  private ArrayList<SwervePod> pods;

  private coordType currentCoordType;
  private coordType lastCoordType;
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

  /* ##################################################################################
   *  BEGIN: Temporary Code for PIDController of rotation to stop drift in AutonCrude
   * ################################################################################## */
    private int arraytrack;
    double[] AngleHist = {0.0, 0.0, 0.0, 0.0, 0.0};
    double autonCrudeGyroAngleAvg;
  /* ##################################################################################
   *  END: Temporary Code for PIDController of rotation to stop drift in AutonCrude
   * ################################################################################## */

   public enum driveMode {
    DEFENSE,
    DRIVE,
    TURBO,
    VISION,
    ORBIT
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
    gyroUpdateOffset();
    updateAngle();
    // SmartDashboard.putNumber("currentAngle", this.currentAngle);

    // SmartDashboard.putNumber("forwardCommand", 0);
    // SmartDashboard.putNumber("strafeCommand", 0);
    // SmartDashboard.putNumber("spinCommand", 0);

    isVisionDriving = false;

  /* ##################################################################################
   *  BEGIN: Temporary Code for PIDController of rotation to stop drift in AutonCrude
   * ################################################################################## */
    arraytrack = 0;
    autonCrudeGyroAngleAvg = 0;
  /* ##################################################################################
   *  END: Temporary Code for PIDController of rotation to stop drift in AutonCrude
   * ################################################################################## */

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

    //this.forwardCommand = SmartDashboard.getNumber("forwardCommand", 0);
    //this.strafeCommand = SmartDashboard.getNumber("strafeCommand", 0);
    //this.spinCommand = SmartDashboard.getNumber("spinCommand", 0);

    // SmartDashboard.putNumber("drive()InputForwardCommand", forwardCommand);
    // SmartDashboard.putNumber("drive()InputStrafeCommand", strafeCommand);
    // SmartDashboard.putNumber("drive()InputSpinCommand", spinCommand);

    

    // TODO: Make the gyro reset if a certain button is pushed
    updateAngle();
    // board.putNumber("Drive updated currentAngle Degrees", (this.currentAngle * 180/Math.PI));
    // SmartDashboard.putString("Drive currentCoordType", currentCoordType.toString());

    if(currentDriveMode != driveMode.TURBO) {
      this.forwardCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
      this.strafeCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
      this.spinCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
    }

    if(currentCoordType == coordType.FIELD_CENTRIC) {
      final double temp = (this.forwardCommand * Math.cos(this.currentAngle) + this.strafeCommand * Math.sin(this.currentAngle));
      this.strafeCommand = (-this.forwardCommand * Math.sin(this.currentAngle) + this.strafeCommand * Math.cos(this.currentAngle));
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
    // SmartDashboard.putNumber("this.forwardComDriveTrain.drive", this.forwardCommand);
    // SmartDashboard.putNumber("this.strafeComDriveTrain.drive", this.strafeCommand);
    // SmartDashboard.putNumber("this.spinComDriveTrain.drive", this.spinCommand);
    calculateNSetPodPositions(this.forwardCommand, this.strafeCommand, this.spinCommand);  
    
    SmartDashboard.putBoolean("isOrbiting", currentDriveMode == driveMode.ORBIT);
    SmartDashboard.putBoolean("isRobotCentric", currentCoordType == coordType.ROBOT_CENTRIC);
    SmartDashboard.putBoolean("isFieldCentric", currentCoordType == coordType.FIELD_CENTRIC);
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

      
      // ###########################################################
      // BEGIN:   Ether Eqns -- Ether's official derivation
      //          +Y :=  axis of chassis forward movement
      //          +X :=  axis of chassis strafe to starboard/right
      // ###########################################################
      double a = strafeCommand - spinCommand * getRadius("A");
      double b = strafeCommand + spinCommand * getRadius("B");;
      double c = forwardCommand - spinCommand * getRadius("C");  
      double d = forwardCommand + spinCommand * getRadius("D");
      
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
      


      /*
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
      */

      // SmartDashboard.putNumber("a", a);
      // SmartDashboard.putNumber("b", b);
      // SmartDashboard.putNumber("c", c);
      // SmartDashboard.putNumber("d", d);
      for (int idx = 0; idx < 4; idx++) {
        // .putNumber("preScale P" + (idx + 1) + " podDrive", podDrive[idx]);
      }

      // Find the highest pod speed then normalize if a pod is exceeding our max speed
      relMaxSpeed = Math.max(Math.max(podDrive[0], podDrive[1]), Math.max(podDrive[2], podDrive[3]));
      if(relMaxSpeed > maxSpeed_InchesPerSec) {
        for(int idx = 0; idx < pods.size(); idx++) {
          podDrive[idx] /= relMaxSpeed / maxSpeed_InchesPerSec;
        }
      }

      // Set calculated drive and spins to each pod
      //for(int idx = 0; idx < pods.size(); idx++) {
        for(int idx = 0; idx < (pods.size()); idx++) {
        pods.get(idx).set(podDrive[idx], podSpin[idx]);   //TODO: try doing pods.size() - 1 in for conditional, then outside for loop
                                                          //  do a hardcode set of pods.get(3).set(0.1, 0.0);
        SmartDashboard.putNumber("pod" + idx + " drive", podDrive[idx]);    
        SmartDashboard.putNumber("pod" + idx + " spin", podSpin[idx]);
        }
        //pods.get(3).set(0.1,1.57);
        

    //} else { // Enter defenseive position
    //  double smallNum = Math.pow(10, -15);
    //  pods.get(0).set(smallNum, -1.0 * Math.PI / 4.0);
    //  pods.get(1).set(smallNum, 1.0 * Math.PI / 4.0);
    //  pods.get(2).set(smallNum, 3.0 * Math.PI / 4.0);
    //  pods.get(3).set(smallNum, -3.0 * Math.PI / 4.0);

    SmartDashboard.putBoolean("orbiting", isOrbiting());
    }
  }

  private void updateAngle() {
    // -pi to pi; 0 = straight
    this.currentAngle = (((((gyro.getAngle()) - this.gyroOffset) * Math.PI/180.0)) % (2*Math.PI));
    // gyro.getAngle is returned in degrees.
    // Then converted to radians via "* pi/180".
    // And finally, it's modulus against 2pi is taken and returned as currentAngle.
  }

  public void gyroUpdateOffset() {
    this.gyroOffset = (gyro.getAngle());
  }

  private double getRadius(String component) {
    // Omitted if driveStatements where we pivoted around a pod
    // This'll be orbit and dosado in the future
    // if(currentDriveMode == driveMode.ORBIT) {
      // if(component.equals("A") || component.equals("B")) { return length / 2.0; }
      // else if(component.equals("C")) { return width; }
      // else /* component D */ { return 2 * width; } // Puts radius to the right of bot at distance w
    // } else {
      if(component.equals("A") || component.equals("B")) { return length / 2.0 ; }
      else { return width / 2.0; }  //TODO: place to check for forward vs back pods working vs not working
    // }
  }

  public void setDriveMode(driveMode wantedDriveMode) {
    currentDriveMode = wantedDriveMode;
  }

  public void setCoordType(coordType wantedType) {
    currentCoordType = wantedType;
  }

  public boolean isOrbiting() {
    if(currentDriveMode == driveMode.ORBIT) { return true; }
    return false;
  }

  public void resetGyro() {
    gyro.reset();
  }

  public coordType getCurrentCoordType() {
    return currentCoordType;
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

/* ##################################################################################
 *  BEGIN: Temporary Code for PIDController of rotation to stop drift in AutonCrude
 * ################################################################################## */
  private void calcAutonCrudeGyroAngleAvg() {
    this.AngleHist[this.arraytrack] = this.currentAngle;
    autonCrudeGyroAngleAvg = ( this.AngleHist[0] + this.AngleHist[1] + this.AngleHist[2] + this.AngleHist[3] + this.AngleHist[4] ) / 5;
  }

  public double getAutonCrudeGyroAngleAvg() {
    return this.autonCrudeGyroAngleAvg;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    calcAutonCrudeGyroAngleAvg();
    this.arraytrack++;
    if (this.arraytrack > 3) {
      this.arraytrack = 0;
    } 
/* ##################################################################################
 *  END: Temporary Code for PIDController of rotation to stop drift in AutonCrude
 * ################################################################################## */


  }

}
