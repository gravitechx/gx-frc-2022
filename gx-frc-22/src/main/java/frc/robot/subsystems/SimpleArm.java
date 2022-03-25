// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SimpleArm extends SubsystemBase {
  CANSparkMax motor = new CANSparkMax(Constants.ARM_NEO_ID, MotorType.kBrushless);
  RelativeEncoder encoder = motor.getEncoder(Type.kHallSensor, 42);

  private static SimpleArm arm;

  public static float ARM_MAX = 0.0f;
  public static double ARM_UP = 0;
  public static float ARM_DOWN = -37.4f;

  
  /** Creates a new SimpleArm. */
  public SimpleArm() {
    motor.restoreFactoryDefaults();

    // set the soft limit.
    motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ARM_MAX);
    motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ARM_DOWN);
    SmartDashboard.putNumber("encoder position", encoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder position", encoder.getPosition());

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

  // Set the soft limit
  public void enableSoftLimit(boolean enabled) {
    // motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, enabled);
    // motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, enabled);
  }

  // Zero Arm Encoder
  public void zeroArmEncoder() {
    encoder.setPosition(0);
  }

  public static SimpleArm getInstance() {
    if (arm == null) arm = new SimpleArm();
    return arm;
  }
}
