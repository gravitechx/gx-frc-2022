// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  //drivetrain constants | deadband might be changed between controller/joystick
  public static final double DEADBAND = 0.1;
  public static final double MAX_OUTPUT = 1.0;
  public static final double VOLT_COMP = 6.0;
  public static final double AMP_LIMIT = 40.0;

  // private wheel =
  // 1.5 feet per wheel rotation

  private static DriveTrain driveTrain;

  //create motor objects
  WPI_TalonFX talonRLeader = new WPI_TalonFX(Constants.TALON_R_LEADER_PORT);
  WPI_TalonFX talonRFollower = new WPI_TalonFX(Constants.TALON_R_FOLLOWER_PORT);
  WPI_TalonFX talonLLeader = new WPI_TalonFX(Constants.TALON_L_LEADER_PORT);
  WPI_TalonFX talonLFollower = new WPI_TalonFX(Constants.TALON_L_FOLLOWER_PORT);

  //create DifferentialDrive object
  DifferentialDrive drivetrain = new DifferentialDrive(talonLLeader, talonRLeader);

  // PID stuff
  double kP = 0.3;
  double kI = 0.0;
  double kD = 0.0;
  PIDController pid = new PIDController(kP, kI, kD);
  
  //Creates a new DriveTrain and sets the deadband and max output according to drivetrain constants
  public DriveTrain() {
    talonLFollower.follow(talonLLeader);
    talonRFollower.follow(talonRLeader);
    talonRLeader.setInverted(true);
    talonRFollower.setInverted(true);
    
    drivetrain.setDeadband(DEADBAND);
    drivetrain.setMaxOutput(MAX_OUTPUT);

    applyChanges();
    setDriveEncoder(0);
  }

  //method to drive in arcade style, throttle joystick is the reversed one
  public void arcadeDrive(double throttle, double turn) {
    drivetrain.arcadeDrive(throttle, turn);
  }

  public void setDriveEncoder(int value) {
    talonRLeader.setSelectedSensorPosition(value);
    talonLLeader.setSelectedSensorPosition(value);
  }

  public void driveToPOS() {
    pid.calculate(talonRLeader.getSensorCollection().getIntegratedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void applyChanges() {
    // Resets motors to factory default
    talonRLeader.configFactoryDefault();
    talonRFollower.configFactoryDefault();
    talonLLeader.configFactoryDefault();
    talonLFollower.configFactoryDefault();

    //sets voltage compensation (very important!)
    talonRLeader.configVoltageCompSaturation(VOLT_COMP);
    talonRFollower.configVoltageCompSaturation(VOLT_COMP);
    talonLLeader.configVoltageCompSaturation(VOLT_COMP);
    talonLFollower.configVoltageCompSaturation(VOLT_COMP);

    // Enables voltage compensation
    talonRLeader.enableVoltageCompensation(true);
    talonRFollower.enableVoltageCompensation(true);
    talonLLeader.enableVoltageCompensation(true);
    talonLFollower.enableVoltageCompensation(true);
   
    //sets input current limit to motors
    talonRLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, AMP_LIMIT, AMP_LIMIT, 0));
    talonRFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, AMP_LIMIT, AMP_LIMIT, 0));
    talonLLeader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, AMP_LIMIT, AMP_LIMIT, 0));
    talonLFollower.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, AMP_LIMIT, AMP_LIMIT, 0));

    talonRLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    talonLLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public static DriveTrain getInstance() {
    if (driveTrain == null) driveTrain = new DriveTrain();
    return driveTrain;
  }

  public WPI_TalonFX getLLeader() {
    return talonLLeader;
  }

  public WPI_TalonFX getRLeader() {
    return talonRLeader;
  }

  public WPI_TalonFX getLFollow() {
    return talonLFollower;
  }

  public WPI_TalonFX getRFollow() {
    return talonRFollower;
  }
}
