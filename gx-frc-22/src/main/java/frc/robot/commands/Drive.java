// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class Drive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain drivetrain;

   /* @param subsystem The subsystem used by this command.
   */
  public Drive(DriveTrain subsystem) {
    drivetrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get values from joystick
    double valueY = -OI.getInstance().getJoystick().getY();
    double valueX = OI.getInstance().getJoystick().getX();
    //double valueY = -OI.getInstance().getController().getRawAxis(1);
    //double valueX = OI.getInstance().getController().getRawAxis(2);

    //use built in method for arcade drive
    drivetrain.arcadeDrive(valueY, valueX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
