// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DriveTrain;

// public class Auto extends CommandBase {
//   /** Creates a new Auto. */
//   // Declare a drivetrain.
//   DriveTrain driveTrain;
//   // Declare a timer.
//   Timer timer;

//   public Auto(DriveTrain subsystem) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     driveTrain = subsystem;
//     addRequirements(driveTrain);
//     timer = new Timer();
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.reset(); 
//     timer.start(); 
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     DriveTrain.getInstance().autoDrive(1);
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
