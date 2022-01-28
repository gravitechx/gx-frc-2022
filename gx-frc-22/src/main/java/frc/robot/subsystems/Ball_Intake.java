// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.subsystems;

//import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ball_Intake extends SubsystemBase {
  /** Creates a new Ball_Intake. */
CANSparkMax front = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
CANSparkMax back = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
// CANEncoder leftEncoder = backLeft.getEncoder();
// CANEncoder rightEncoder = backRight.getEncoder();

  public Ball_Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotor()
  {
    //set percent output
  }

  public void stopMotor()
  {
    //stop
  }
}
