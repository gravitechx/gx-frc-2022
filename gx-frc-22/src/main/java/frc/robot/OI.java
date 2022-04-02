package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    //define which button or joystick is used here 
    private static final int DRIVE_THROTTLE_AXIS = 1;
  
    private static final int DRIVE_TURN_AXIS = 4;
    //buttons below are PLACEHOLDERS
    private static final int ARM_UP_PORT = 3;
    private static final int ARM_DOWN_PORT = 2;

    private static final double ARM_UP_SPEED = 0.55;
    private static final double ARM_DOWN_SPEED = -0.25;

    private static final double DEADBAND_UP = 0.1;
    private static final double DEADBAND_DOWN = 0.1; 

    // The backup controller joyostick ports
    // forward backward = 1 (y axis) (reverse)
    // left right = 2 (z axis)
    
    // The set speed for the d-pad
    // private static final double DRIVE_SET_SPEED = 0.20;

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
        return Math.signum(gameController.getRawAxis(DRIVE_THROTTLE_AXIS)) * -Math.pow(gameController.getRawAxis(DRIVE_THROTTLE_AXIS), power);
    }

    public double getTurnAxis() {
        return 0.43 * Math.signum(gameController.getRawAxis(DRIVE_TURN_AXIS)) * Math.pow(gameController.getRawAxis(DRIVE_TURN_AXIS), power);
    }

    public double getArmUp() {
        double speed = 0;
        if (gameController.getRawAxis(ARM_UP_PORT) > DEADBAND_UP) speed = gameController.getRawAxis(ARM_UP_PORT) * ARM_UP_SPEED;
        else speed = 0;

        return speed;
    }

    public double getArmDown() {
        double speed = 0;
        if (gameController.getRawAxis(ARM_DOWN_PORT) > DEADBAND_DOWN) speed = gameController.getRawAxis(ARM_DOWN_PORT) * ARM_DOWN_SPEED;
        else speed = 0;

        return speed;
    }

    // Return the set speed for the drive when the dpad is pressed. Is not used.
    // public double getDriveSetSpeed() {
    //     return DRIVE_SET_SPEED;
    // }

    /* Singleton: if you call OI object, this methos will use an object that already exists,
    and if none exists it will create a new instance of the object. Now you can say "getInstance"
    instead of creating a new instance every time you need one */
    public static OI getInstance() {
        if (oi == null) oi = new OI();
        return oi;
    }
}