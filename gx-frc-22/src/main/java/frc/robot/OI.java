package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.DriveToPositionMeters;


public class OI {
    private static OI oi;
    private Joystick drivecontroller;

    private static final int BUTTON_1 = 1;
    private static final int BUTTON_2 = 2;
    private static final int BUTTON_3 = 3;
    private static final int BUTTON_4 = 4;



    private OI () {
        drivecontroller = new Joystick(0);
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(drivecontroller, BUTTON_1).whileHeld(new DriveToPositionMeters(1.0, -1.0, 0), true); // left turn
        new JoystickButton(drivecontroller, BUTTON_2).whileHeld(new DriveToPositionMeters(-1.0, -1.0, 0), true); // drive back
        new JoystickButton(drivecontroller, BUTTON_3).whileHeld(new DriveToPositionMeters(-1.0, 1.0, 0), true); // right turn
        new JoystickButton(drivecontroller, BUTTON_4).whileHeld(new DriveToPositionMeters(1.0, 1.0, 0), true); // drive forward
    }

    public double getDriveX () {
        return drivecontroller.getZ();
    }

    public double getDriveY () {
        return -drivecontroller.getY();
    }

    public Joystick getDriveController() {
        return drivecontroller;
    }
   
    public static OI getInstance() {
        if (oi == null) oi = new OI();
        return oi;
    }
}
