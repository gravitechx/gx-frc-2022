// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SimpleArm extends SubsystemBase {
  CANSparkMax motor = new CANSparkMax(Constants.ARM_NEO_ID, MotorType.kBrushless);
  private static SimpleArm arm;

  /** Creates a new SimpleArm. */
  public SimpleArm() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinArm(double speed) {
    motor.set(speed);
  }

  public void autoArm(double speed, long time) {
    long start = System.currentTimeMillis();
    long end = start + time;

    while(System.currentTimeMillis() < end) {
      motor.set(speed);
    }
  }

  public static SimpleArm getInstance() {
    if (arm == null) arm = new SimpleArm();
    return arm;
  }
}
