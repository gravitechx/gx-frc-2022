package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;

//import com.revrobotics.CANEncoder;
// Person of unkown identity(***redacted***) had name idea
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



  /** Creates a new Ball_Intake. */
public class EverybotIntake extends SubsystemBase {
static private CANSparkMax leader = new CANSparkMax(5, MotorType.kBrushless);

static EverybotIntake intake;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void ballIn()
  {
    leader.set(0.7);
  }

  public void ballOut()
  {
    leader.set(-0.7);
  }

  // Singleton - makes only one instance of BallIntake
  public static EverybotIntake getInstance(){
    if(intake == null)
    {
      intake = new EverybotIntake();
    }

    return intake;
  }

  

  }

