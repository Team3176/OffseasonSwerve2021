package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.constants.ControllerConstants;

public class Controller {
    private static Controller instance = new Controller();

    private final Joystick transStick;
    private final Joystick rotStick;

    // Drivetrain buttons
    private final JoystickButton orbitButton;
    private final JoystickButton dosadoButton;
    private final JoystickButton visionButton;
    private final JoystickButton defenseButton;
    private final JoystickButton fieldCentricButton;
    private final JoystickButton robotCentricButton;
    private final JoystickButton backRobotCentricButton;

    public static Controller getInstance() { return instance; }

    public Controller() {
        transStick = new Joystick(ControllerConstants.TRANSLATION_STICK_ID);
        rotStick = new Joystick(ControllerConstants.ROTATION_STICK_ID);

        // All buttons numbers subject to change
        orbitButton = new JoystickButton(rotStick, 2);
        dosadoButton = new JoystickButton(rotStick, 3);
        defenseButton = new JoystickButton(transStick, 2);
        visionButton = new JoystickButton(transStick, 3); //Should be part of the xbox controller later
        fieldCentricButton = new JoystickButton(transStick, 4);
        robotCentricButton = new JoystickButton(transStick, 5);
        backRobotCentricButton = new JoystickButton(transStick, 6);
    }

    public double getForward() { return transStick.getY(); }
    public double getStrafe() { return transStick.getX(); }
    public double getSpin() { return rotStick.getX(); }    

    public void outputToSmartDashboard() { 
        SmartDashboard.putNumber("forward", getForward());
        SmartDashboard.putNumber("strafe", getStrafe());
        SmartDashboard.putNumber("spin", getSpin());
        

    }

    /* Buttons */

    public JoystickButton getOrbitButton() { return orbitButton; }

    public JoystickButton getDosadoButton() { return dosadoButton; }

    public JoystickButton getVisionButton() { return visionButton; }

    public JoystickButton getDefenseButton() { return defenseButton; }

    public boolean isFieldCentricButtonPressed() { return fieldCentricButton.get(); }

    public boolean isRobotCentricButtonPressed() { return robotCentricButton.get(); }    

    public boolean isBackRobotCentricButtonPressed() { return backRobotCentricButton.get(); }
}