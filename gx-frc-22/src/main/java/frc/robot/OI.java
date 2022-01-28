package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
    private static OI oi;
    private Joystick joystickX;
    private Joystick joystickY;

    private OI () {
        joystickX = new Joystick(1);
        joystickY = new Joystick(0);
    }

    public Joystick getJoystickX () {
        return joystickX;
    }

    public Joystick getJoystickY () {
        return joystickY;
    }
   
    public static OI getInstance() {
        if (oi == null) oi = new OI();
        return oi;
    }
}
