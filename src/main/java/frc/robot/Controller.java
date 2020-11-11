package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.ControllerConstants;

public class Controller {
    private static Controller instance = new Controller();

    private final Joystick transStick;
    private final Joystick rotStick;

    public static Controller getInstance() {
        return instance;
    }

    public Controller() {
        transStick = new Joystick(ControllerConstants.TRANSLATION_STICK_ID);
        rotStick = new Joystick(ControllerConstants.ROTATION_STICK_ID);
    }

    public double getRotStickX() { return rotStick.getX(); }
    public double getTransStickX() { return transStick.getX(); }
    public double getTransStickY() { return transStick.getY(); }

    /*  
        When at 45 degrees the magnitide is radical 2
        Therefore, this was normalized since max output
        in X or Y meant the driver wanted max speed whatever given angle
    */
    public double getTransStickMag() {
        double magSquared = (getTransStickX() * getTransStickX()) + (getTransStickY() * getTransStickY());
        if(Math.sqrt(magSquared) > 1) {
            return 1 * ControllerConstants.SLOW_DRIVE_MULT;
        }
        return Math.sqrt(magSquared) * ControllerConstants.SLOW_DRIVE_MULT;
    }

    //This might not work because it might give us radians
    public double getTransStickAngleRads() {
        /*
            Atan2 yields -1 to 1 radians where the highest magnitude is on the left of the circle
            To flip to the right side, make x negative then add Pi to shift everyhting to positive
        */
        double angle = Math.atan2(getTransStickY(), -getTransStickX());
        return angle + Math.PI;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("transStickX", getTransStickX());
        SmartDashboard.putNumber("transStickY", getTransStickY());
        SmartDashboard.putNumber("transStickMag", getTransStickMag());
    }
}