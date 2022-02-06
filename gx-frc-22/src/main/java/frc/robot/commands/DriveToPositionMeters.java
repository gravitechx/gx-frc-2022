// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToPositionMeters extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain drivetrain;

    private double tolerance;
    private double lefttarget,righttarget;

    /*
     * @param subsystem The subsystem used by this command.
     */
    public DriveToPositionMeters(double distanceR, double distanceL, double tolerance) {
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);

        this.tolerance = tolerance;
        lefttarget = distanceL;
        righttarget = distanceR;


    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.zeroDriveEncoders();
        drivetrain.resetPositionPID();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.setPositionPIDMeters(righttarget, lefttarget);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.getTalonRLeader().set(ControlMode.PercentOutput, 0);
        drivetrain.getTalonLLeader().set(ControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double Lerror = drivetrain.getLPositionPIDErrorMeters();
        double Rerror = drivetrain.getRPositionPIDErrorMeters();
        if (Math.abs(Lerror) < tolerance && Math.abs(Rerror) < tolerance) { // if the controller is within the tolerance
            return true;
        }
        return false;
    }
}
