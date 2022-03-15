// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArmUp extends CommandBase {

    private static final double movementDistance = 100;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveTrain drivetrain;

    private double tolerance;
    private double distance;

    /*
     * @param subsystem The subsystem used by this command.
     */

    public ArmUp(double distance, double tolerance) {
        drivetrain = DriveTrain.getInstance();
        addRequirements(drivetrain);

        this.tolerance = tolerance;
        distance = movementDistance;

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
        drivetrain.setPositionPIDMeters(distance);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.getTalonRLeader().set(ControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double uperror = drivetrain.getLPositionPIDErrorMeters();
        if (Math.abs(uperror) < tolerance) { // if the controller is within the tolerance
            return true;
        }
        return false;
    }
}
