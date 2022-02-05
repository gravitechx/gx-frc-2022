// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveVelocityMetersPerSecond extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain drivetrain;

    /*
     * @param subsystem The subsystem used by this command.
     */
    public DriveVelocityMetersPerSecond(double velR, double velL) {
        drivetrain = Drivetrain.getInstance();
        addRequirements(drivetrain);

        // drivetrain.zeroDriveEncoders(); // not necessary for velocity control
        drivetrain.resetVelocityPID();
        drivetrain.setVelocityPIDMetersPerSecond(velR, velL);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.updateVelocityPID();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.resetVelocityPID();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
