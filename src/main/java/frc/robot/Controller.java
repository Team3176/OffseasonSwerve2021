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
    private final JoystickButton reZeroGyroButton;
    private final JoystickButton slalomButton;

    public static Controller getInstance() { return instance; }

    public Controller() {
        transStick = new Joystick(ControllerConstants.TRANSLATION_STICK_ID);
        rotStick = new Joystick(ControllerConstants.ROTATION_STICK_ID);

        // All buttons numbers subject to change
        orbitButton = new JoystickButton(transStick, 1);
        dosadoButton = new JoystickButton(rotStick, 3);
        defenseButton = new JoystickButton(transStick, 2);
        visionButton = new JoystickButton(transStick, 3); //Should be part of the xbox controller later
        fieldCentricButton = new JoystickButton(transStick, 4);
        robotCentricButton = new JoystickButton(transStick, 5);
        backRobotCentricButton = new JoystickButton(transStick, 6);
        reZeroGyroButton = new JoystickButton(rotStick, 4);
        slalomButton = new JoystickButton(transStick, 7);
    }

    public double getForward() { 
        if (Math.abs(-transStick.getY())<.06) {
            return 0.0;
        } else {
            return 1 * (Math.pow((transStick.getY()),1)*1);} // 1 for 2021, -1 for 2019
    }
    
    public double getStrafe() { 
        if (Math.abs(transStick.getX())<.06) {
            return 0.0;
        } else {
            return 1 * (Math.pow((-1 * transStick.getX()),1) * 1);} // 1 for 2021, -1 for 2019
    }

    public double getSpin() { 
        if (Math.abs(rotStick.getX())<.06) {
            return 0.0;
        } else {
            return -1 * Math.pow(rotStick.getX(),1) / 7.0;} // -1 for 2021, 1 for 2019
    }

    public int getPOVTransStick() {
        return transStick.getPOV();
    }

    public void outputToSmartDashboard() {     
        
    }

    /* Buttons */

    public JoystickButton getOrbitButton() { return orbitButton; }

    public JoystickButton getDosadoButton() { return dosadoButton; }

    public JoystickButton getVisionButton() { return visionButton; }

    public JoystickButton getDefenseButton() { return defenseButton; }

    public JoystickButton getReZeroGyroButton() {return reZeroGyroButton; }

    public JoystickButton getSlalomButton() {return slalomButton; }

    public boolean isFieldCentricButtonPressed() { return fieldCentricButton.get(); }

    public boolean isRobotCentricButtonPressed() { return robotCentricButton.get(); }    

    public boolean isBackRobotCentricButtonPressed() { return backRobotCentricButton.get(); }
}