package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;

//import com.revrobotics.CANEncoder;
// Person of unkown identity(***redacted***) had name idea
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



  /** Creates a new Ball_Intake. */
public class BallIntake extends SubsystemBase {
static private CANSparkMax follower = new CANSparkMax(3, MotorType.kBrushless);
static private CANSparkMax leader = new CANSparkMax(5, MotorType.kBrushless);
static BallIntake intake;



 
//This is making neos follow each other and is making the follower spin in the opposite direction to the leader
  public BallIntake() {
    //Setting motors to follow and inverting to make them go the same way
    follower.follow(leader);
    follower.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  //This the method that will run the motor speed is |0.7|
  public void runMotor()
  {
    leader.set(0.7);
  }

  // Singleton - makes only one instance of BallIntake
  public static BallIntake getInstance(){
    if(intake == null)
    {
      intake = new BallIntake();
    }

    return intake;
  }

  

  }

