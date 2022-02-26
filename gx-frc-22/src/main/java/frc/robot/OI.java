package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    //define which button or joystick is used here 
    private static final int DRIVE_THROTTLE_AXIS = 1;
    private static final int DRIVE_TURN_AXIS = 2;
    //buttons below are PLACEHOLDERS
    private static final int BALL_INTAKE_BUTTON = 0;
    private static final int BALL_OUTTAKE_BUTTON = 0;
    private static final int ARM_UP_BUTTON = 6;
    private static final int ARM_DOWN_BUTTON = 7;

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
        return -gameController.getRawAxis(DRIVE_THROTTLE_AXIS);
    }

    public double getTurnAxis() {
        return gameController.getRawAxis(DRIVE_TURN_AXIS);
    }

    //hold intake button to spin balls inward
    public boolean getIntakeButton() {
        return gameController.getRawButton(BALL_INTAKE_BUTTON);
    }

    //hold outtake button to spin balls outward
    public boolean getOuttakeButton() {
        return gameController.getRawButton(BALL_OUTTAKE_BUTTON);
    }

    public boolean ArmUpButton() {
        return gameController.getRawButtonPressed(ARM_UP_BUTTON);
    }

    public boolean ArmDownButton() {
        return gameController.getRawButtonPressed(ARM_DOWN_BUTTON);
    }

    //add your button and methods of those buttons here, define your buttons up top

    /* Singleton: if you call OI object, this methos will use an object that already exists,
    and if none exists it will create a new instance of the object. Now you can say "getInstance"
    instead of creating a new instance every time you need one */
    public static OI getInstance() {
        if (oi == null) oi = new OI();
        return oi;
    }
}
