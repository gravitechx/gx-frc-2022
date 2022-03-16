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
    CANSparkMax motor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    RelativeEncoder encoder = motor.getEncoder(Type.kHallSensor, 4096);
   

    SparkMaxPIDController controller = motor.getPIDController();

    // Gains
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kF = 0;
    double kIZone = 0;
    
   

    public Arm() {
        encoder.setPositionConversionFactor(Constants.PULSE_PER_REVOLUTION);

        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setFF(kF);
        controller.setIZone(kIZone);

        controller.setOutputRange(-1, 1);
    }

    public void drive(double f, double t) {
        
    }

    public void PID(double distance) {
        controller.setReference(distance, ControlType.kPosition);
        
    }

    @Override
    public void periodic() {

    }
}
