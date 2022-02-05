// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Util;

public class Drivetrain extends SubsystemBase {

  private static Drivetrain drivetrain;
  private PIDController LPIDcontroller;
  private PIDController RPIDcontroller;

  private static final double WHEEL_DIAMETER = 0.1524; // wheel diameter in meters
  private static final double GEAR_RATIO = 10.71; // motor rotations : wheel rotations
  private static final double MOTOR_ENCODER_EPR = 2048; // encoder ticks per revolution
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  // meters * ( wheelrotation/meters * motorrotation/wheelrotation * encoderticks/motorrotation ) = encoderticks
  public static final double METER_TO_ENCODER = 1/WHEEL_CIRCUMFERENCE * GEAR_RATIO * MOTOR_ENCODER_EPR;
  // converts meters per second to encoder ticks per 100ms
  public static final double MPS_TO_ENCP100MS = 0.1*METER_TO_ENCODER;

  private static final double DEADBAND = 0.1;
  private static final double MAX_OUTPUT = 1;
  private static final double MAX_SPEED = 3; // 3m/s max

  private static final double MAX_DRIVE_SUPPLY_CURRENT = 40.0; // amps
  private static final double SUPPLY_CURRENT_TRIP_TIME = 0.5; // seconds
  private static final double VOLTAGE_COMP_VALUE = 10.0; // volts
  private static final int _TIMEOUT = 10; // CAN timeout in ms

  // these constants are small because they are based on an error in raw encoder units!
  private static final double PID_POS_P = 5.1e-6; // proportional
  private static final double PID_POS_I = 0.0; // integral
  private static final double PID_POS_D = 0.0; // derivative

  private static final double PID_VEL_S = 0.04; // static adder
  private static final double PID_VEL_F = 0.33; // feed-forward in output/meters_per_second
  private static final double PID_VEL_P = 0.0;
  private static final double PID_VEL_I = 0.0;
  private static final double PID_VEL_D = 0.0;

  WPI_TalonFX talonRLeader = new WPI_TalonFX(Constants.TALON_R_LEADER_PORT);
  WPI_TalonFX talonRFollower = new WPI_TalonFX(Constants.TALON_R_FOLLOWER_PORT);
  WPI_TalonFX talonLLeader = new WPI_TalonFX(Constants.TALON_L_LEADER_PORT);
  WPI_TalonFX talonLFollower = new WPI_TalonFX(Constants.TALON_L_FOLLOWER_PORT);

  DifferentialDrive diffdrive = new DifferentialDrive(talonLLeader, talonRLeader);

  // Creates a new DriveTrain
  public Drivetrain() {
    diffdrive.setDeadband(DEADBAND);
    diffdrive.setMaxOutput(MAX_OUTPUT);

    applyToAllDrive((motor) -> motor.configFactoryDefault(_TIMEOUT)); // reset all configs

    // set up followers and inversions
    talonLFollower.follow(talonLLeader);
    talonRFollower.follow(talonRLeader);
    talonRLeader.setInverted(true);
    talonRFollower.setInverted(true);

    // config current limit in amps to be at the limit with a trip time in seconds
    SupplyCurrentLimitConfiguration supplylim = new SupplyCurrentLimitConfiguration(true, MAX_DRIVE_SUPPLY_CURRENT,
        MAX_DRIVE_SUPPLY_CURRENT, SUPPLY_CURRENT_TRIP_TIME);
    applyToAllDrive((motor) -> motor.configSupplyCurrentLimit(supplylim));

    // config and enable voltage compensation
    // the motor will output higher % when battery is below the saturation value and lower % when over
    applyToAllDrive((motor) -> motor.configVoltageCompSaturation(VOLTAGE_COMP_VALUE));
    applyToAllDrive((motor) -> motor.enableVoltageCompensation(true));

    // slow down status frames we don't use to reduce CAN bus load
    applyToAllDrive((motor) -> motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255));
    applyToAllDrive((motor) -> motor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 255));
    applyToAllDrive((motor) -> motor.setStatusFramePeriod(StatusFrame.Status_6_Misc, 255));
    applyToAllDrive((motor) -> motor.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, 255));
    applyToAllDrive((motor) -> motor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255));
    applyToAllDrive((motor) -> motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255));
    applyToAllDrive((motor) -> motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255));
    applyToAllDrive((motor) -> motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255));
    applyToAllDrive((motor) -> motor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255));
    applyToAllDrive((motor) -> motor.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 255));
    applyToAllDrive((motor) -> motor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255));
    
    talonRLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    talonRLeader.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
    talonRLeader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    talonLLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    talonLLeader.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
    talonLLeader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    applyToAllDrive((motor) -> motor.setSelectedSensorPosition(0)); // zero encoder

    // set up PID controllers. These can be set to velocity PID later if needed.
    LPIDcontroller = new PIDController(PID_POS_P, PID_POS_I, PID_POS_D);
    LPIDcontroller.disableContinuousInput();
    RPIDcontroller = new PIDController(PID_POS_P, PID_POS_I, PID_POS_D);
    RPIDcontroller.disableContinuousInput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void arcadeDrive(double throttle, double turn) {
    diffdrive.arcadeDrive(throttle, turn);
  }

  public void arcadeDriveSquared(double throttle, double turn) {
    diffdrive.arcadeDrive(throttle, turn, true);
  }

  public void zeroDriveEncoders() {
    talonRLeader.setSelectedSensorPosition(0);
    talonLLeader.setSelectedSensorPosition(0);
  }

  // resets the position PID controller integral/error and sets the output to 0%
  public void resetPositionPID() {
    LPIDcontroller = new PIDController(PID_POS_P, PID_POS_I, PID_POS_D);
    RPIDcontroller = new PIDController(PID_POS_P, PID_POS_I, PID_POS_D);
    RPIDcontroller.reset();
    LPIDcontroller.reset();
    talonRLeader.set(ControlMode.PercentOutput, 0);
    talonLLeader.set(ControlMode.PercentOutput, 0);
  }

  // sets the position PID controller targets
  public void setPositionPIDMeters(double Rtarget, double Ltarget) {
    RPIDcontroller.setSetpoint(Rtarget * METER_TO_ENCODER);
    LPIDcontroller.setSetpoint(Ltarget * METER_TO_ENCODER);
  }

  // return the PID controller error in meters
  public double getRPositionPIDErrorMeters() {
    // dividing by constant converts from encoder units to meters
    return RPIDcontroller.getPositionError() / METER_TO_ENCODER; 
  }

  public double getLPositionPIDErrorMeters() {
    // dividing by constant converts from encoder units to meters
    return LPIDcontroller.getPositionError() / METER_TO_ENCODER; 
  }

  public void updatePositionPID() {
    double Rcom = RPIDcontroller.calculate(talonRLeader.getSelectedSensorPosition());
    double Lcom = LPIDcontroller.calculate(talonLLeader.getSelectedSensorPosition());
    talonRLeader.set(ControlMode.PercentOutput, Rcom);
    talonLLeader.set(ControlMode.PercentOutput, Lcom);
  }

  // resets the velocity PID controller integral/error and sets the output to 0%
  public void resetVelocityPID() {
    LPIDcontroller = new PIDController(PID_VEL_P, PID_VEL_I, PID_VEL_D);
    RPIDcontroller = new PIDController(PID_VEL_P, PID_VEL_I, PID_VEL_D);
    RPIDcontroller.reset();
    LPIDcontroller.reset();
    talonRLeader.set(ControlMode.PercentOutput, 0);
    talonLLeader.set(ControlMode.PercentOutput, 0);
  }

    // return the PID controller error in meters
    public double getRVelocityPIDErrorMeters() {
      // dividing by constant converts from encoder units to meters/second
      return RPIDcontroller.getPositionError() / MPS_TO_ENCP100MS; 
    }
  
    public double getLVelocityPIDErrorMeters() {
      // dividing by constant converts from encoder units to meters/second
      return LPIDcontroller.getPositionError() / MPS_TO_ENCP100MS; 
    }

  // sets the velocity PID controller targets
  public void setVelocityPIDMetersPerSecond(double Rtarget, double Ltarget) {
    Rtarget = Util.constrain(Rtarget, -MAX_SPEED, MAX_SPEED); // constrain speed to specified limits
    Ltarget = Util.constrain(Ltarget, -MAX_SPEED, MAX_SPEED);
    RPIDcontroller.setSetpoint(Rtarget * MPS_TO_ENCP100MS); // sets setpoint in meters per 100ms
    LPIDcontroller.setSetpoint(Ltarget * MPS_TO_ENCP100MS);
  } 

  // update the PID and set motor output
  public void updateVelocityPID() {
    double Rcom = RPIDcontroller.calculate(talonRLeader.getSelectedSensorVelocity());
    double Lcom = LPIDcontroller.calculate(talonLLeader.getSelectedSensorVelocity());
    Rcom += RPIDcontroller.getSetpoint()/MPS_TO_ENCP100MS*PID_VEL_F; // velocity feedforward
    Rcom += Math.signum(Rcom) * PID_VEL_S; // static feedforward
    Lcom += RPIDcontroller.getSetpoint()/MPS_TO_ENCP100MS*PID_VEL_F;
    Lcom += Math.signum(Lcom) * PID_VEL_S;
    talonRLeader.set(ControlMode.PercentOutput, Rcom); // push outputs to motor controllers
    talonLLeader.set(ControlMode.PercentOutput, Lcom);
  }

  public PIDController getLPIDController() {
    return LPIDcontroller;
  }

  public PIDController getRPIDController() {
    return RPIDcontroller;
  }

  // getters for the drivemotors in case we need their encoder values or something
  public WPI_TalonFX getTalonRLeader() {
    return talonRLeader;
  }

  public WPI_TalonFX getTalonRFollower() {
    return talonRFollower;
  }

  public WPI_TalonFX getTalonLLeader() {
    return talonLLeader;
  }

  public WPI_TalonFX getTalonLFollower() {
    return talonLFollower;
  }

  // a Java lamba that applies a setting to all of something. See the constructor of this class for usage. 
  // Copied from HarkerRoboLib HSSwerve.
  public void applyToAllDrive(Consumer<WPI_TalonFX> consumer) {
    consumer.accept(talonRLeader);
    consumer.accept(talonLLeader);
    consumer.accept(talonRFollower);
    consumer.accept(talonLFollower);
  }

  public static Drivetrain getInstance() {
    if (drivetrain == null) {
      drivetrain = new Drivetrain();
    }
    return drivetrain;
  }

}
