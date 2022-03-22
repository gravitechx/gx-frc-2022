// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallIntake;


public class SpinningStickOut extends CommandBase {

  int value = 0;

  /** Creates a new SpinningStick. */
  public SpinningStickOut() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(BallIntake.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    BallIntake.getInstance().ballOut();
  }

  // Called once the command ends or is interrupted.
  @Override

  public void end(boolean interrupted) {
    BallIntake.getInstance().setSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}