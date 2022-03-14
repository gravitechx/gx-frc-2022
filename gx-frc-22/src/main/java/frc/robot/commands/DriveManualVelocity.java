// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.OI;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveManualVelocity extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain drivetrain;

   /* @param subsystem The subsystem used by this command.
   */
  public DriveManualVelocity() {
    drivetrain = DriveTrain.getInstance();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrain.resetVelocityPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get values from joysticks
    double valueY = OI.getInstance().getThrottleAxis();
    double valueX = OI.getInstance().getTurnAxis();

    // square inputs and convert to m/s
    valueY = Math.pow(valueY, 2) * Math.signum(valueY) * DriveTrain.MAX_SPEED;
    valueX = Math.pow(valueX, 2) * Math.signum(valueX) * DriveTrain.MAX_SPEED;

    //use built in method for arcade drive
    DriveTrain.setVelocityPIDMetersPerSecond(valueY, valueX);
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
