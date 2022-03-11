// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.IntakeArm;

public class PositionArm extends CommandBase {
  /** Creates a new PositionArm. */
  public PositionArm() {
    addRequirements(IntakeArm.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  //everybot22
  @Override
  public void execute() {
    IntakeArm.getInstance().Spin();
      //if(OI.getInstance().ArmUpButton())
      //((IntakeArm) IntakeArm.getInstance()).rotateDegrees(0);
      // Added cast to method reciever thing, dont treally know what it does, but fixes error!
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