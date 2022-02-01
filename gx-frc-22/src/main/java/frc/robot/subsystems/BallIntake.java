package frc.robot.subsystems;

//import com.revrobotics.CANEncoder;
// Person of unkown identity(***redacted***) had name idea
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

  /** Creates a new Ball_Intake. */
public class BallIntake extends SubsystemBase {
static private CANSparkMax follower = new CANSparkMax(3, MotorType.kBrushless);
static private CANSparkMax leader = new CANSparkMax(5, MotorType.kBrushless);
static BallIntake intake;
//creates pneumatics objects and their parameters 
DoubleSolenoid left = new DoubleSolenoid(0, null, 0, 0);
DoubleSolenoid right = new DoubleSolenoid(0, null, 0, 0);
//find the exact motor ID for our neos
 
// CANEncoder leftEncoder = backLeft.getEncoder();
// CANEncoder rightEncoder = backRight.getEncoder();
//This is making neos follow each other and is making the follower spin in the opposite direction to the leader
  public BallIntake() {
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
//this is the method that will bring the pneumatics upwards 
  public void pneumoUP(){
    right.set(Value.kForward);
    left.set(Value.kForward);
  }
//this is the method that will bring the pneumatics down
  public void pneumoDOWN(){
    right.set(Value.kReverse);
    left.set(Value.kReverse);
  }

}
