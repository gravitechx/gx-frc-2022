// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// DEPRECATED

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
//import frc.robot.subsystems.BallIntake;

public class IntakeArm extends SubsystemBase {
  //instantiates motor
  static private CANSparkMax armLeader = new CANSparkMax(4, MotorType.kBrushless);
  static IntakeArm arm; 
  // remember to change the motor ID^

  // Values need to be changed when hardware is availible for testing
  private final double Kp = 0.0;
  private final double Ki = 0.0;
  private final double Kd = 0.0;
  // these are the constants that need to later be changed

  public final PIDController turnController;

  public IntakeArm() {
    turnController = new PIDController(Kp, Ki, Kd, 0.02);

    // takes all the inputs from the PID Controller

    // turnController.setTolerance(2.0f);
    // this is the tolerance, which is the acceptable amount of error (it can't be
    // perfect, like it is on the graph
    // DUMMY VALS THAT NEED TO BE CHANGED^^^
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // gets motor position from encoder
  public void IntakePosition() {
    addRequirements(IntakeArm.getInstance());
    armLeader.getEncoder().getPosition();

  }

  private void addRequirements(Subsystem subsystem) {
  }

  //Passes the output directly to setSetpoint().
  public void pidWrite(double Output) {
    armLeader.set(0.3);
  }

  //rotate motor to the angle we want (described by "rotation")
  public void rotateDegrees(double rotation) {
    armLeader.getEncoder().setPosition(0);
    turnController.reset();
    turnController.setPID(Kp, Ki, Kd);
    // sets the values again, we have this above, but putting it here again just in case
    turnController.setSetpoint(rotation);
  }
  // the amount (in degrees) that we want the motor to rotate(the length we want
  // the arm to go up/down)

    //TEMPORARY TESTER
  public void Spin(double speed) {
    armLeader.set(speed);
  }

  public static IntakeArm getInstance() {
    if (arm == null) arm = new IntakeArm();
    return arm;
  }

}
