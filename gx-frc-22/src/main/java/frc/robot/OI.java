package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    //define which button or joystick is used here 
    private static final int DRIVE_THROTTLE_AXIS = 1;
  
    private static final int DRIVE_TURN_AXIS = 4;
    //buttons below are PLACEHOLDERS
    private static final int BALL_INTAKE_BUTTON = 4;
    private static final int BALL_OUTTAKE_BUTTON = 5;
    private static final int ARM_UP_BUTTON = 0;
    private static final int ARM_DOWN_BUTTON = 6;

    public double power = 2;

    private static OI oi;
    private XboxController gameController;

    //create instance of gamepad
    private OI () {
        gameController = new XboxController(Constants.CONTROLLER_PORT);
    }

    public XboxController getController() {
        return gameController;
    }

    //return joystick inputs for driving
    public double getThrottleAxis() {
        return Math.signum(gameController.getRawAxis(DRIVE_THROTTLE_AXIS)) * Math.pow(gameController.getRawAxis(DRIVE_THROTTLE_AXIS), power);
    }

    public double getTurnAxis() {
        return 0.5 * Math.signum(gameController.getRawAxis(DRIVE_TURN_AXIS)) * Math.pow(gameController.getRawAxis(DRIVE_TURN_AXIS), power);
    }
    /* Singleton: if you call OI object, this methos will use an object that already exists,
    and if none exists it will create a new instance of the object. Now you can say "getInstance"
    instead of creating a new instance every time you need one */
    public static OI getInstance() {
        if (oi == null) oi = new OI();
        return oi;
    }
  
    public boolean getArmUpButton() {
        return false;
    }
}