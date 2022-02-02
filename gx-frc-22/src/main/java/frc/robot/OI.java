//This class stores all operator inpute suchas joysticks and buttons.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
    private static OI oi;
    private Joystick joystick;
    private XboxController gameController;
    

    //create instance of joystick or gamepad
    private OI () {
        joystick = new Joystick(0);
        //gameController = new XboxController(0);
    }

    /*public XboxController getController() {
        return gameController;
    }*/

    //methods that return joystick values
    public Joystick getJoystick () {
        return joystick;
    }
   
    /* Singleton: if you call OI object, this method will use an object that already exists,
    and if none exists it will create a new instance of the object. Now you can say "getInstance"
    instead of creating a new instance every time you need one. */
    public static OI getInstance() {
        if (oi == null) oi = new OI();
        return oi;
    }
}