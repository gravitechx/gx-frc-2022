
// USE

package frc.robot.subsystems;

//import com.revrobotics.CANEncoder;
// Person of unkown identity(***redacted***) had name idea
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

  /** Creates a new Ball_Intake. */
public class BallIntake extends SubsystemBase {
  static private CANSparkMax leader = new CANSparkMax(Constants.BALL_INTAKE_MOTOR, MotorType.kBrushless);
  static BallIntake intake;

  public BallIntake() {
    // Set a current limit of 30 amps.
    leader.setSmartCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  public void ballIn()
  {
    leader.set(Constants.SPIN_SPEED);
  }
  public void ballOut()
  {
    leader.set(Constants.SPIN_SPEED_REVERSE);
  }

  public void setSpeed(double speed)
  {
    leader.set(speed);
  }

  // amazing time logic for auto
  public void autoSpin(double speed, long time) {
    long start = System.currentTimeMillis();
    long end = start + time;

    while(System.currentTimeMillis() < end) {
      leader.set(speed);
    }

    leader.set(0);
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

