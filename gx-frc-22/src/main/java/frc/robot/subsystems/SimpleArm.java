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

  // declare arm.
  private static SimpleArm arm;

  // Set limits for the arm encoding.
  public static float ARM_MAX = -0.1f;
  public static double ARM_UP = 0;
  public static float ARM_DOWN = -36.4f;

  
  /** Creates a new SimpleArm. */
  public SimpleArm() {

    // Reset all motor settings
    motor.restoreFactoryDefaults();

    // set the soft limit.
    motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ARM_MAX);
    motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ARM_DOWN);
    motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    SmartDashboard.putNumber("encoder position", encoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder position", encoder.getPosition());

    // Get the soft limit
    SmartDashboard.putNumber("Soft Limit Forward", motor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
    SmartDashboard.putNumber("Soft Limit Reverse", motor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));
  }

  // Set the speed of the arm manually
  public void manualArm(double speed) {
    motor.set(speed);
  }

  // Auto
  public void autoArm(double speed, long time) {
    long start = System.currentTimeMillis();
    long end = start + time;

    while(System.currentTimeMillis() < end) {
      motor.set(speed);
    }
  }

  // enable the soft limit
  public void enableSoftLimit(boolean enabled) {
    motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, enabled);
    motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, enabled);
  }

  // Zero Arm Encoder
  public void zeroArmEncoder() {
    encoder.setPosition(0);
  }

  public RelativeEncoder getEncoder() {
    return encoder;
  }

  public static SimpleArm getInstance() {
    if (arm == null) arm = new SimpleArm();
    return arm;
  }
}
