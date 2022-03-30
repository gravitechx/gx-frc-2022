// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.SimpleArm;

public class ManualArm extends CommandBase {
  /** Creates a new ManualArm. */
  public ManualArm() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SimpleArm.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Arm Up", OI.getInstance().getArmUp());
    SmartDashboard.putNumber("Arm Down", OI.getInstance().getArmDown());

    if (OI.getInstance().getArmUp() > 0 || OI.getInstance().getArmDown() < 0) {
      SmartDashboard.putString("inside if?", "yes");
      if (OI.getInstance().getArmUp() > 0) {
        SimpleArm.getInstance().manualArm(OI.getInstance().getArmUp());
      }
      if (OI.getInstance().getArmDown() < 0) {
        SimpleArm.getInstance().manualArm(OI.getInstance().getArmDown());
      }
    } else {
      SmartDashboard.putString("inside if?", "nah");
      SimpleArm.getInstance().manualArm(0.02);
    }
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
