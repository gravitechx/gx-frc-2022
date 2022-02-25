// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.BallIntake;
//creates new arm
public class IntakeArm extends SubsystemBase {
  static private CANSparkMax armLeader= new CANSparkMax(5, MotorType.kBrushless);
  static IntakeArm arm; 
  // remember to change the motor ID^

  // Values need to be changed when hardware is availible for testing
  private final double Kp = 0.03;
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

  // gets arm position from encoder
  public void IntakeDown() {
    addRequirements(IntakeArm.getInstance());
    armLeader.getEncoder().getPosition();

  }

  private void addRequirements(Subsystem subsystem) {
  }

  //
  public void pidWrite(double Output) {
    //set(ControlType.kPosition, Output);
  }

  // sets the output
  private void set(ControlType percentoutputControlType, double d, double Output) {
  }

  public void rotateDegrees(double rotation) {
    //IntakeArm.getEncoder().setPosition(0);
    turnController.reset();
    turnController.setPID(Kp, Ki, Kd);
    // sets the values again, we have this above, but putting it here again just in
    // case
    turnController.setSetpoint(rotation);
    // turnController.
    // the point that we want the arm to end up at
    // turnController.start
    // turns on the controller

  }
  // the amount (in degrees) that we want the motor to rotate(the length we want
  // the arm to go up/down)

  public static Subsystem getInstance() {
    return null;
  }

}
