package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import frc.robot.constants.ControllerConstants;

public class Controller {
    private final Joystick transStick;
    private final Joystick rotStick;

    private static Controller instance = new Controller();

    public static Controller getInstance() { return instance; }

    public Controller() {
        transStick = new Joystick(ControllerConstants.TRANSLATION_STICK_ID);
        rotStick = new Joystick(ControllerConstants.ROTATION_STICK_ID);
    }
    
    public double getForward() { return transStick.getY(); }
    public double getStrafe() { return transStick.getX(); }
    public double getSpin() { return rotStick.getX(); }     
}