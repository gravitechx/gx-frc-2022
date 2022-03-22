// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.


// // PID UNUSED.


// package frc.robot.commands;
 
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import frc.robot.Constants;
// import frc.robot.subsystems.Arm;

// public class ArmUp extends CommandBase {

//     @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
//     private final Arm arm;

//     private double tolerance;
//     private double rotations;

//     /*
//      * @param subsystem The subsystem used by this command.
//      */

//     public ArmUp(double rotations, double tolerance) {
//         arm = Arm.getInstance();
//         addRequirements(arm);

//         this.tolerance= tolerance;
//         this.rotations = rotations;
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         System.out.println("Armupcalled");
//      //arm.ZeroArmEncoder();
//        // drivetrain.resetPositionPID();
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         arm.setrotations(this.rotations);
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         arm.getMotor().set(0);
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         double uperror = rotations-arm.getencoderposition();
//         if (Math.abs(uperror) < tolerance) { // if the controller is within the tolerance
//             return true;
//         }
//         return false;
//     }
// }
