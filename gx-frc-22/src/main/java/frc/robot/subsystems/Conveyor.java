// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Conveyor extends SubsystemBase {
  
  // New motor controllers for motors.

  // Make sure motor type is brushed or not.
  private static CANSparkMax leftConveyorLead = new CANSparkMax(Constants.LEFT_CONVEYOR_PORT, MotorType.kBrushless);
  private static CANSparkMax rightConveyorFollow = new CANSparkMax(Constants.RIGHT_CONVEYOR_PORT, MotorType.kBrushless);

  // Create new instance for singleton
  private static Conveyor instance;

  /** Creates a new Conveyor. */
  public Conveyor() {

    // Right motor follows left. Invert right motor.
    rightConveyorFollow.follow(leftConveyorLead);
    rightConveyorFollow.setInverted(true);

  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Set the speed.
  public void setMotorSpeed(double speed) {
    leftConveyorLead.set(speed);
  }

  //Singleton to make sure there is only one instance of Conveyor.
  public static Conveyor getInstance() {
    if (instance == null) {
      instance = new Conveyor();
    }
    
    return instance;
  }
}
