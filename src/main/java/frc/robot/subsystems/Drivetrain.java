package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.DrivetrainConstants;
import frc.robot.Controller;

import java.util.ArrayList;
public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance = new Drivetrain();
  private Controller controller = Controller.getInstance();

  private AHRS gyro;

  private ArrayList<SwervePod> pods;

  public TalonFX[] driveControllers = {new TalonFX(DrivetrainConstants.DRIVE_ONE_CID),
                                            new TalonFX(DrivetrainConstants.DRIVE_TWO_CID),
                                            new TalonFX(DrivetrainConstants.DRIVE_THREE_CID),
                                            new TalonFX(DrivetrainConstants.DRIVE_FOUR_CID)};

  public TalonSRX[] spinControllers = {new TalonSRX(DrivetrainConstants.STEER_ONE_CID),
                                        new TalonSRX(DrivetrainConstants.STEER_TWO_CID),
                                        new TalonSRX(DrivetrainConstants.STEER_THREE_CID),
                                        new TalonSRX(DrivetrainConstants.STEER_FOUR_CID)};

  private double length;
  private double width;

  private double maxSpeed;
  private double relMaxSpeed;

  private double currentAngle;

  private SwervePod podFR;
  private SwervePod podFL;
  private SwervePod podBL;
  private SwervePod podBR;

  // Prevents more than one instance of drivetrian
  public static Drivetrain getInstance() { return instance; }

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

    // Setting constants
    length = DrivetrainConstants.LENGTH;
    width = DrivetrainConstants.WIDTH;
    maxSpeed = DrivetrainConstants.MAX_ROBOT_SPEED;

    // Instantiating the gyro
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
    updateAngle();
  }

  /**
   * @param forwardCommand range of {-1,1}
   * @param strafeCommand range of {-1, 1}
   * @param spinCommand range of {-1, 1}
   */
  public void drive(double forwardCommand, double strafeCommand, double spinCommand) {

    updateAngle();
    // TODO: Not sure if the following three lines are used in swerve or should just be commented
    double temp = forwardCommand * Math.sin(currentAngle) + strafeCommand * Math.cos(currentAngle);
    strafeCommand = (-forwardCommand * Math.cos(currentAngle) + strafeCommand * Math.sin(currentAngle));
    forwardCommand = temp;

    // Create arrays for the speed and angle of each pod
    double[] podDrive = new double[4];
    double[] podSpin = new double[4];

    double a = strafeCommand + spinCommand * (length / 2.0);
    double b = strafeCommand - spinCommand * (length / 2.0);
    double c = forwardCommand - spinCommand * (width / 2.0);
    double d = forwardCommand + spinCommand * (width / 2.0);

    // Calculate speed and angle of each pod
    podDrive[0] = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));
    podSpin[0] = Math.atan2(b, c);

    podDrive[1] = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
    podSpin[1] = Math.atan2(b, d);

    podDrive[2] = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));
    podSpin[2] = Math.atan2(a, d);

    podDrive[3] = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));
    podSpin[3] = Math.atan2(a, c);

    // Pod drives should be -1 to 1 so normalize if greater than that
    relMaxSpeed = Math.max(Math.max(podDrive[0], podDrive[1]), Math.max(podDrive[2], podDrive[3]));
    if(relMaxSpeed > 1) {
      for(int i = 0; i < pods.size(); i++) {
        podDrive[i] /= relMaxSpeed;
      }
    }

    // Turn podDrive vales into speeds then set calculated drive and spins to each pod
    for(int i = 0; i < pods.size(); i++) {
      podDrive[i] *= maxSpeed;
      pods.get(i).set(podDrive[i], podSpin[i]);   
    }
  } 

  private void updateAngle() {
    currentAngle = ((((gyro.getAngle() + 90) * Math.PI/180.0)) % (2*Math.PI));
  }
}
