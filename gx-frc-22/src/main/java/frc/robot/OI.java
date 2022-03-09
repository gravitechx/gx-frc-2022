//This class stores all operator inpute suchas joysticks and buttons.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import java.lang.Math;

public class OI {
    //define which button or joystick is used here
    private static final int DRIVE_THROTTLE_AXIS = 1;
    private static final int DRIVE_TURN_AXIS = 4;

    private static OI oi;
    private XboxController gameController;
    
    //create instance of gamepad
    private OI () {
        gameController = new XboxController(0);
    }

    public XboxController getController() {
        return gameController;
    }

    //return joystick inputs for driving, squared to reduce sensitivity
    public double getThrottleAxis() {
        return -Math.pow(gameController.getRawAxis(DRIVE_THROTTLE_AXIS), 5);
    }

    public double getTurnAxis() {
        return Math.pow(gameController.getRawAxis(DRIVE_TURN_AXIS), 5);
    }

    //add your buttons and methods of those buttons here, define your buttons up top


    /* Singleton: if you call OI object, this method will use an object that already exists,
    and if none exists it will create a new instance of the object. Now you can say "getInstance"
    instead of creating a new instance every time you need one. */
    public static OI getInstance() {
        if (oi == null) oi = new OI();
        return oi;
    }
}