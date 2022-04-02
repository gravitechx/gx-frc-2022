// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

//import frc.robot.commands.SpinningStickIn;
//import frc.robot.commands.SpinningStickOut;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.MoveArm;
import frc.robot.commands.SpinningStickIn;
import frc.robot.commands.SpinningStickOut;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.SimpleArm;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoGroup;
import frc.robot.commands.DriveSetSpeed;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain autoSubsystem = DriveTrain.getInstance();
  private final SimpleArm arm = SimpleArm.getInstance();
  private final BallIntake ballIntake = BallIntake.getInstance();
  private final AutoGroup autoCommand = new AutoGroup();

  private static final double DRIVE_SET_SPEED = 0.30;
  
  /*
    arm up = 3
    armdown = 2
    spin in = 6
    spin out = 5

    forward backward = 1 (y axis) (reverse)
    left right = 2 (z axis)
  */

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton intakeIn = new JoystickButton(OI.getInstance().getController(), 6);
    JoystickButton intakeOut = new JoystickButton(OI.getInstance().getController(), 5); // 4 = Y, 3 = X can be changed
    JoystickButton armUp = new JoystickButton(OI.getInstance().getController(), 2);
    JoystickButton armDown = new JoystickButton(OI.getInstance().getController(), 1);

    // dpad buttons, set speed
    JoystickButton setSpeedForward = new JoystickButton(OI.getInstance().getController(), 4);
    JoystickButton setSpeedBackward = new JoystickButton(OI.getInstance().getController(), 3);

    intakeIn.whileHeld(new SpinningStickIn(ballIntake));
    intakeOut.whileHeld(new SpinningStickOut(ballIntake));

    armUp.whileHeld(new MoveArm(arm, 0.55));
    armDown.whileHeld(new MoveArm(arm, -0.25));

    // Move forward at set speed.
    // Uses the same drive object as auto, is that fine?
    setSpeedForward.whileHeld(new DriveSetSpeed(autoSubsystem, 0.50));
    setSpeedBackward.whileHeld(new DriveSetSpeed(autoSubsystem, -0.30));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }

}
