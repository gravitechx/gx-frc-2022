// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  DoubleSolenoid left = new DoubleSolenoid(0, null, 0, 0);
  DoubleSolenoid right = new DoubleSolenoid(0, null, 0, 0);
  /** Creates a new pneumatics. */
  public Pneumatics() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
