// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  //drivetrain constants
  public static final double DEADBAND = 0.1;
  public static final double MAX_OUTPUT = 1;

  private static DriveTrain driveTrain;

  //create motor objects
  WPI_TalonFX talonRLeader = new WPI_TalonFX(Constants.TALON_R_LEADER_PORT);
  WPI_TalonFX talonRFollower = new WPI_TalonFX(Constants.TALON_R_FOLLOWER_PORT);
  WPI_TalonFX talonLLeader = new WPI_TalonFX(Constants.TALON_L_LEADER_PORT);
  WPI_TalonFX talonLFollower = new WPI_TalonFX(Constants.TALON_L_FOLLOWER_PORT);

  //create DifferentialDrive object
  DifferentialDrive drivetrain = new DifferentialDrive(talonLLeader, talonRLeader);
  
  //Creates a new DriveTrain and sets the deadband and max output according to drivetrain constants
  public DriveTrain() {
    talonLFollower.follow(talonLLeader);
    talonRFollower.follow(talonRLeader);
    
    drivetrain.setDeadband(DEADBAND);
    drivetrain.setMaxOutput(MAX_OUTPUT);
  }

  //method to drive in arcade style, throttle joystick is the reversed one
  public void arcadeDrive(double throttle, double turn) {
    drivetrain.arcadeDrive(throttle, turn);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static DriveTrain getInstance() {
    if (driveTrain == null) driveTrain = new DriveTrain();
    return driveTrain;
  }
}
