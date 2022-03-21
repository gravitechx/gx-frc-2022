// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(Constants.ARM_NEO_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    RelativeEncoder encoder = motor.getEncoder(Type.kHallSensor, 42);
    

    // Gains
     private static double kP = 0.03;
     private static double kI = 0;
     private static double kD = 0;
     private static double kF = 0;
     private static double kIZone = 0;
     public static double ARM_MAX = 41.5;
     public static double ARM_UP = 37.4;
     public static double ARM_DOWN = 0.04;

    private static Arm arm;

    SparkMaxPIDController controller = motor.getPIDController();    

    public Arm() {
      motor.restoreFactoryDefaults();
      // encoder.setPositionConversionFactor(Constants.PULSE_PER_REVOLUTION);
      motor.setSmartCurrentLimit(80);
      SparkMaxPIDController controller = motor.getPIDController();
      
      controller.setP(kP);
      controller.setI(kI);
      controller.setD(kD);
      controller.setFF(kF);
      controller.setIZone(kIZone);

      controller.setOutputRange(-1, 1);
      
      
      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIZone);
      SmartDashboard.putNumber("encoder position",encoder.getPosition());
      SmartDashboard.putNumber("SetPoint", 0);
    }

    public void setrotations(double rotations) {
    controller.setReference(rotations, CANSparkMax.ControlType.kPosition);

  
    }
    

    public double getencoderposition () {
      return encoder.getPosition();
    }

    @Override
    public void periodic() {
      //read PID coefficeints from SmartDashboard 
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I zone", 0);
    double rotations = SmartDashboard.getNumber("SetPoint", 0);
    
    //if PID coeffeicnts on SmartDashboard have changed, write ne values to controller 
    if((p != kP)){
    controller.setP(p); kP=p;}
    if((i != kI)) {
      controller.setP(i); kI=i;}
    if((d != kD)) {
      controller.setD(d); kD=d;}
    if((iz != kIZone)) {
      controller.setIZone(iz); kIZone=p;}
    if((p != kP)) {
      controller.setP(p); kP=p;}

    
    SmartDashboard.putNumber("encoder position",encoder.getPosition());
    SmartDashboard.putNumber("SetPoint", rotations);
    

    }

    public CANSparkMax getMotor(){
      return motor;
    }

  

    public void ZeroArmEncoder() {
      encoder.setPosition(0);
    }

    public static Arm getInstance() {
      if (arm == null) {
        arm = new Arm();
      }
      return arm;
    }
}
