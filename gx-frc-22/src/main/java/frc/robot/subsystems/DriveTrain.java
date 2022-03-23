// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Util;

public class DriveTrain extends SubsystemBase {

  private static DriveTrain drivetrain;


  private static final double MOTOR_ENCODER_EPR = 2048; // encoder ticks per revolution
  // meters * ( wheelrotation/meters * motorrotation/wheelrotation * encoderticks/motorrotation ) = encoderticks
  // converts meters per second to encoder ticks per 100ms



 

  private static final double MAX_DRIVE_SUPPLY_CURRENT = 40.0; // amps
  private static final double SUPPLY_CURRENT_TRIP_TIME = 0.5; // seconds
  private static final double VOLTAGE_COMP_VALUE = 6.0; // volts
  private static final int _TIMEOUT = 100; // CAN timeout in ms

  // the PID controller outputs -1028 to 1028
  // error of 100 encoder ticks with a O of 0.1 results in an output of 10/1028, ~1%.
  private static final int PID_POS_SLOT = 0; // PID constants slot, 0-3
  //private static final double PID_POS_S = 0.04; // static adder for stiction
  private static final double PID_POS_P = 0.021; // proportional
  private static final double PID_POS_I = 0.0001; // integral
  private static final double PID_POS_D = 0.01; // derivative

  private static final int PID_VEL_SLOT = 1;
  private static final double PID_VEL_S = 0.04;
  private static final double PID_VEL_F = 0.0472; // feed-forward in output/ticks_per_100ms
  private static final double PID_VEL_P = 0.0;
  private static final double PID_VEL_I = 0.0;
  private static final double PID_VEL_D = 0.0;

  
  static private CANSparkMax ArmUp = new CANSparkMax(4, MotorType.kBrushless)


 

  /* Creates a new DriveTrain
  public DriveTrain() {
    
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
    talonLLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    talonLLeader.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
    applyToAllDrive((motor) -> motor.setSelectedSensorPosition(0)); // zero encoder

    // set up PID controllers. These can be set to velocity PID later if needed.
    applyToAllDrive((motor) -> motor.config_kP(PID_POS_SLOT, PID_POS_P, _TIMEOUT));
    applyToAllDrive((motor) -> motor.config_kI(PID_POS_SLOT, PID_POS_I, _TIMEOUT));
    applyToAllDrive((motor) -> motor.config_kD(PID_POS_SLOT, PID_POS_D, _TIMEOUT));
    applyToAllDrive((motor) -> motor.config_kF(PID_POS_SLOT, 0, _TIMEOUT)); // no kF for position

    applyToAllDrive((motor) -> motor.config_kP(PID_VEL_SLOT, PID_VEL_P, _TIMEOUT));
    applyToAllDrive((motor) -> motor.config_kI(PID_VEL_SLOT, PID_VEL_I, _TIMEOUT));
    applyToAllDrive((motor) -> motor.config_kD(PID_VEL_SLOT, PID_VEL_D, _TIMEOUT));
    applyToAllDrive((motor) -> motor.config_kF(PID_VEL_SLOT, PID_VEL_F, _TIMEOUT));
  */

// Normal Code
package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {
  //drivetrain constants
  public static final double DEADBAND = 0.05;
  public static final double MAX_OUTPUT = 1.0;
  public static final double VOLT_COMP = 6.0;
  public static final double AMP_LIMIT = 40.0;
  public double amount = 0.0;

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
    drivetrain.arcadeDrive(throttle, turn, false);
  }

  public void autoDrive(double throttle, double turn, long time) {
    long start = System.currentTimeMillis();
    long end = start + time;

    while(System.currentTimeMillis() < end) {
      SmartDashboard.putNumber("Amount", amount++);
      arcadeDrive(throttle, turn);
    }
  }

  public void stop() {
    talonLLeader.set(0);
    talonRLeader.set(0);
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

  /*
  Stuff added to Arm. Not used.
  public void zeroDriveEncoders() {
    talonRLeader.setSelectedSensorPosition(0);
  
  }

  // resets the position PID controller integral/error and sets the output to 0%


  // sets the position PID controller targets
  
  public void setPositionPIDMeters(double Rtarget) {
   // talonRLeader.set(ControlMode.Position, Rtarget * METER_TO_ENCODER,
     // DemandType.ArbitraryFeedForward, PID_POS_S*Math.signum(talonRLeader.getClosedLoopError())); 
    //talonLLeader.set(ControlMode.Position, Ltarget * METER_TO_ENCODER,
     // DemandType.ArbitraryFeedForward, PID_POS_S*Math.signum(talonRLeader.getClosedLoopError())); 
  }

  

  // sets the velocity PID controller targets
  public void setVelocityPIDMetersPerSecond(double Rtarget, double Ltarget) {
    Rtarget = Util.constrain(Rtarget, -MAX_SPEED, MAX_SPEED); // keep within limits
    double Rcom = Math.signum(Rtarget) * PID_VEL_S; // static feedforward
    

    // command motor controllers
   
    
  } 

  // getters for the drivemotors in case we need their encoder values or something
  

  // a Java lamba that applies a setting to all of something. See the constructor of this class for usage. 
  // Copied from HarkerRoboLib HSSwerve.
  

  public static DriveTrain getInstance() {
    if (drivetrain == null) {
      drivetrain = new DriveTrain();
    }
    return drivetrain;
  }

}
*/

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