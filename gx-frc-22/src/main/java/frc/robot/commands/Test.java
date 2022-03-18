// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Test extends CommandBase {
  /** Creates a new Test. */
  public Test() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(Arm.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Arm Auto", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Arm.getInstance().test(100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Arm Auto", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Arm.getInstance().getController().getPositionError()) < 100; // Tolerance
  }
}
