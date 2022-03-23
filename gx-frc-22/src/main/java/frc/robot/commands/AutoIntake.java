// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallIntake;

public class AutoIntake extends CommandBase {
  private double speed;
  private long time;

  /** Creates a new AutoIntake. */
  public AutoIntake(double speed, long time) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(BallIntake.getInstance());

    this.speed = speed;
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intake is running");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    BallIntake.getInstance().autoSpin(speed, time);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
