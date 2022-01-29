//This class stores all operator inpute suchas joysticks and buttons.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
    private static OI oi;
    private Joystick joystickX;
    private Joystick joystickY;

    //create instance of joystick or gamepad
    private OI () {
        joystickX = new Joystick(1);
        joystickY = new Joystick(0);
    }

    //methods that return joystick values
    public Joystick getJoystickX () {
        return joystickX;
    }

    public Joystick getJoystickY () {
        return joystickY;
    }
   
    /* Singleton: if you call OI object, this method will use an object that already exists,
    and if none exists it will create a new instance of the object. Now you can say "getInstance"
    instead of creating a new instance every time you need one. */
    public static OI getInstance() {
        if (oi == null) oi = new OI();
        return oi;
    }
}