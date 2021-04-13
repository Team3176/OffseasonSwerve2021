package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {

  private RobotContainer robotContainer;

  private Controller controller = Controller.getInstance();

  private Command m_autonomousCommand;
  File f;
	public BufferedWriter bw;
	FileWriter fw;

  @Override
  public void robotInit() {
    try {
      f = new File("/home/lvuser/Output.txt");
      if(!f.exists()){
        f.createNewFile();
      }
    fw = new FileWriter(f);
  } catch (IOException e) {
    // TODO Auto-generated catch block
    e.printStackTrace();
  }
    bw = new BufferedWriter(fw);
    robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
  }

  @Override
  public void disabledInit() { }

  @Override
  public void disabledPeriodic() { }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = robotContainer.getAutonomousCommand();


    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
   }

  @Override
  public void autonomousPeriodic() {

   }

  @Override
  public void teleopInit() { }

  @Override
  public void teleopPeriodic() {
    try {
			bw.write("Hellow, I'm a text file");
			bw.close();
			fw.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

    controller.outputToSmartDashboard();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() { }


  
  
}
