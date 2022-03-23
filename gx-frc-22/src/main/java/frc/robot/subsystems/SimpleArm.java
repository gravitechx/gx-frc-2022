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

  public static float ARM_MAX = 40;
  public static double ARM_UP = 37.4;
  public static double ARM_DOWN = 0.04;

  
  /** Creates a new SimpleArm. */
  public SimpleArm() {

    // set the soft limit.
    motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ARM_MAX);
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

  public static SimpleArm getInstance() {
    if (arm == null) arm = new SimpleArm();
    return arm;
  }
}
