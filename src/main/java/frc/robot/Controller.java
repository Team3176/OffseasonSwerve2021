package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.constants.ControllerConstants;

public class Controller {
    private static Controller instance = new Controller();

    private final Joystick transStick;
    private final Joystick rotStick;

    private final JoystickButton swerveNeutralButton;
    private final JoystickButton orbitButton;
    private final JoystickButton dosadoButton;

    public static Controller getInstance() {
        return instance;
    }

    public Controller() {
        transStick = new Joystick(ControllerConstants.TRANSLATION_STICK_ID);
        rotStick = new Joystick(ControllerConstants.ROTATION_STICK_ID);

        swerveNeutralButton = new JoystickButton(transStick, 1); // Subject to change
        orbitButton = new JoystickButton(rotStick, 2); // Subject to change
        dosadoButton = new JoystickButton(rotStick, 3); // Subject to change
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

    public double getRotStickRads() {
        double angle = getRotStickX() * 2 * Math.PI;
        SmartDashboard.putNumber("angle in controller", angle);
        return angle;
    }

    public void outputToSmartDashboard() {
    }

    public JoystickButton getSwerveNeutralButton() {
        return swerveNeutralButton;
    }

    public JoystickButton getOrbitButton() {
        return orbitButton;
    }

    public JoystickButton getDosadoButton() {
        return dosadoButton;
    }

    public boolean defenseEnabled() {
        return transStick.getRawButton(7); // ID an/or stick may change
    }
}