// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private static final int ARM_MOTOR_PORT = 0;
  private static final double VOLT_COMP_ARM = 0;
  private static final double AMP_LIMIT_ARM = 0;
  private static final int ARM_SPARKMAX_PORT = 0;

  // New falcon motor.
  private WPI_TalonFX armMotor = new WPI_TalonFX(ARM_MOTOR_PORT);

  // New sparkmax

  CANSparkMax armSpark = new CANSparkMax(ARM_SPARKMAX_PORT, MotorType.kBrushless);

  /** Creates a new Arm. */
  public Arm() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Set speed for the talon.
  public void setPhoenix(double speed) {
    armMotor.set(ControlMode.PercentOutput, speed);
  }

  public void turnPhoenix(double )

  // Set speed for spark
  public void setSpark(double speed) {
    armSpark.set(speed);
  }


  public void applyChanges() {

    // Changes for talon.
    armMotor.configFactoryDefault();

    armMotor.configVoltageCompSaturation(VOLT_COMP_ARM);

    armMotor.enableVoltageCompensation(true);

    armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, AMP_LIMIT_ARM, AMP_LIMIT_ARM, 0));

    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }
}
