package frc.robot.subsystems;

//import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//Some more imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

  /** Creates a new Ball_Intake. */
public class Ball_Intake extends SubsystemBase {
CANSparkMax leader = new CANSparkMax(3, MotorType.kBrushless);
CANSparkMax follower = new CANSparkMax(5, MotorType.kBrushless);
//find the exact motor ID for our neos
 
// CANEncoder leftEncoder = backLeft.getEncoder();
// CANEncoder rightEncoder = backRight.getEncoder();

  public Ball_Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void robotInit() {
    leader.follow(follower);
  }

  public void runMotor()
  {
    
  }

}
