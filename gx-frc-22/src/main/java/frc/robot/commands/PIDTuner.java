// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.


// // PID? UNUSED


// package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Arm;


// public class PIDTuner extends CommandBase {
//   @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
//     private final Arm drivetrain;

//     private double rotation;
//     private double distance;

//     /*
//      * @param subsystem The subsystem used by this command.
//      */

//     public PIDTuner(double rotation) {
//         drivetrain = Arm.getInstance();
//         addRequirements(drivetrain);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {SmartDashboard.putNumber("encoder position",0);}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
