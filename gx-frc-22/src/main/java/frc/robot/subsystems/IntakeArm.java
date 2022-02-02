// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallIntake;
import java.util.Timer;
import java.util.TimerTask;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeArm extends SubsystemBase implements PIDOutput {
  /** Creates a new IntakeArm. */

  private static final Command timer = null;
private static CANSparkMax armFollower = new CANSparkMax(3, MotorType.kBrushless);
static private CANSparkMax armLeader = new CANSparkMax(5, MotorType.kBrushless);
//remember to change the motor ID
static BallIntake downArm;

private final double Kp = 0.03;
private final double Ki = 0.0;
private final double Kd = 0.0;


turnController = new PIDController(Kp, Ki, Kd, source, output);
turnController.setInputRange(-180.0f, 180.0f);
turnController.setOutputRange(-0.45, 0.45);
turnController.setAbsouluteTolerance(2.0f);
turnController.setContinuous();
}



public final PIDController turnController;
  public IntakeArm() {}

  armFollower.follow(armLeader);
  armFollower.setInverted(true);


    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void IntakeDown(){

    addRequirements(BallIntake.getInstance());
    armLeader.getEncoder().getPosition();
  }

  private void addRequirements(BallIntake instance) {
  }
 
  
  public void pidWrite (double Output){

    set(ControlType.kPosition, -Output, Output);
  }

  private void set(ControlType percentoutputControlType, double d, double Output) {
  }()

  public void rotateDegrees(double rotation){

    
   }

}
