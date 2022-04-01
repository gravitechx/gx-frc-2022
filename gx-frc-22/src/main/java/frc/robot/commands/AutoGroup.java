// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGroup extends SequentialCommandGroup {
  /** Creates a new AutoGroup. */
  public AutoGroup() {
    // Add your commands in the addCommands() call, e.g.
    // add0Commands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoIntake(-0.7, 10000),
      new AutoDrive(-0.5, 0, 1100, 10000)
      // new AutoArm() 
    );
  }
}
