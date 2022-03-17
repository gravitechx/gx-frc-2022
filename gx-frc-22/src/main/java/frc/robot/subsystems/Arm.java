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

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    CANSparkMax motor = new CANSparkMax(Constants.NEO_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

    RelativeEncoder encoder = motor.getEncoder(Type.kHallSensor, 42);

    private static Arm arm;

    SparkMaxPIDController controller = motor.getPIDController();

    // Gains
    double kP = 0.021;
    double kI = 0.0001;
    double kD = 0.01;
    double kF = 0.0472;
    double kIZone = 0.04;
    
   

    public Arm() {
        encoder.setPositionConversionFactor(Constants.PULSE_PER_REVOLUTION / 3);

        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setFF(kF);
        controller.setIZone(kIZone);

        controller.setOutputRange(-1, 1);
    }

    public void PID(double distance) {
        controller.setReference(distance, ControlType.kPosition);
    }

    public double PIDError() {
      return encoder.getPosition() / 3;
    }

    @Override
    public void periodic() {
    
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
