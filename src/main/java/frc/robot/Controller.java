package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.constants.ControllerConstants;

public class Controller {
    // Instantiating the instance of Controller to be used system-wide
    private static Controller instance = new Controller();

    // Instantiating necessary joysticks and buttons
    private final Joystick speedStick = new Joystick(ControllerConstants.SPEED_STICK_ID);
    private final Joystick rotStick = new Joystick(ControllerConstants.ROT_STICK_ID);
    //private final XboxController operator = new XboxController(ControllerConstants.OPERATOR_ID);

    // getInstance function for controller
    public static Controller getInstance() {
        return instance;
    }

    // Gets the joystick speed input for normal arcade drive
    public double getArcadeDriveSpeed() {
        // Switches between a fast and slow drive control by multiplying joystick output by a constant
        if (!speedStick.getRawButton(1)) {
            // Adds in deadband logic
            if (Math.abs(speedStick.getY()) < ControllerConstants.DEADBAND) {
                return 0.0;
            } else if (speedStick.getY() < 0) {
                return (speedStick.getY() + ControllerConstants.DEADBAND) * ControllerConstants.SLOW_SPEED_MULT * (10.0 / 9.0);
            } else {
                return (speedStick.getY() - ControllerConstants.DEADBAND) * ControllerConstants.SLOW_SPEED_MULT  * (10.0 / 9.0);
            }
        } else {
            // Adds in deadband logic
            if (Math.abs(speedStick.getY()) < ControllerConstants.DEADBAND) {
                return 0.0;
            } else if (speedStick.getY() < 0) {
                return (speedStick.getY() + ControllerConstants.DEADBAND) * (10.0 / 9.0);
            } else {
                return (speedStick.getY() - ControllerConstants.DEADBAND) * (10.0 / 9.0);
            }
        }
    }

    // Gets the joystick rotation input for normal arcade drive
    public double getArcadeDriveRot() {
        if(!speedStick.getRawButton(1)){
            // Adds in deadband logic
            if (Math.abs(rotStick.getX()) < ControllerConstants.DEADBAND) {
                return 0.0;
            } else if (rotStick.getX() < 0) {
                return (rotStick.getX() + ControllerConstants.DEADBAND) * ControllerConstants.SLOW_ROT_MULT * Math.PI;
            } else {
                return (rotStick.getX() - ControllerConstants.DEADBAND) * ControllerConstants.SLOW_ROT_MULT * Math.PI;
            }
        } else {
            // Adds in deadband logic
            if (Math.abs(rotStick.getX()) < ControllerConstants.DEADBAND) {
                return 0.0;
            } else if (rotStick.getX() < 0) {
                return (rotStick.getX() + ControllerConstants.DEADBAND) * Math.PI;
            } else {
                return (rotStick.getX() - ControllerConstants.DEADBAND) * Math.PI;
            }
        }
    }

    public Boolean resetOdometry() {
        return speedStick.getRawButton(6);
    }
}