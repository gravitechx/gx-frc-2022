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
import frc.robot.subsystems.BallIntake;

public class IntakeArm extends SubsystemBase{
  /** Creates a new IntakeArm. */

private static CANSparkMax armFollower = new CANSparkMax(3, MotorType.kBrushless);
static private CANSparkMax armLeader = new CANSparkMax(5, MotorType.kBrushless);
//remember to change the motor ID^

//WHAT IS THIS? investigate at ater date
static BallIntake downArm;

//Values need to be changed when hardware is availible for testing
private final double Kp = 0.03;
private final double Ki = 0.0;
private final double Kd = 0.0;
//these are the constants that need to later be changed





public final PIDController turnController;
  public IntakeArm() {
    //Following motor and inverting to make them sync
    armFollower.follow(armLeader);
    armFollower.setInverted(true);

    turnController = new PIDController(Kp, Ki, Kd, 0.02);
//takes all the imputs from the PID Controller


// turnController.setTolerance(2.0f);
//this is the tolerance, which is the acceptable amount of error (it can't be perfect, like it is on the graph)


//DUMMY VALS THAT NEED TO BE CHANGED^^^
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void IntakeDown(){
    addRequirements(BallIntake.getInstance());
    armLeader.getEncoder().getPosition();
  
  }
  //gets the position from the encoder 

  private void addRequirements(BallIntake instance) {
  }
 
  
  public void pidWrite (double Output){
    set(ControlType.kPosition, -Output, Output);
  }
  //the position of motors

  private void set(ControlType percentoutputControlType, double d, double Output) 
  {

  }
  //sets the output 

  public void rotateDegrees(double rotation)
  {
    armFollower.getEncoder().setPosition(0);
    turnController.reset();
    turnController.setPID(Kp, Ki, Kd);
    //sets the values again, we have this above, but putting it here again just in case
    turnController.setSetpoint(rotation);
    //turnController.
    //the point that we want the arm to end up at
   // turnController.start
    //turns on the controller
    
  }
   //the amount (in degrees) that we want the motor to rotate(the length we want the arm to go up/down)

}
