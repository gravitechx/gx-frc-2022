package frc.robot.subsystems;

//import com.revrobotics.CANEncoder;
// Person of unkown identity(***redacted***) had name idea
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

  /** Creates a new Ball_Intake. */
public class BallSuck extends SubsystemBase {
CANSparkMax leader = new CANSparkMax(3, MotorType.kBrushless);
CANSparkMax follower = new CANSparkMax(5, MotorType.kBrushless);
//find the exact motor ID for our neos
 
// CANEncoder leftEncoder = backLeft.getEncoder();
// CANEncoder rightEncoder = backRight.getEncoder();

  public BallSuck() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void robotInit() {
    //leader.follow(follower);
  }

  public void runMotor()
  {
    leader.set(0.7);
    follower.set(-0.7);
  }

}
