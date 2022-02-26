package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
    public final  CANSparkMax m_intakeFront = new CANSparkMax(ShooterConstants.intake, MotorType.kBrushed);
    private final CANSparkMax m_conveyorMiddle = new CANSparkMax(ShooterConstants.conveyor, MotorType.kBrushed);
    private final CANSparkMax m_shooterEnd = new CANSparkMax(ShooterConstants.shooter, MotorType.kBrushless);    
  
    public DigitalInput limitSwitch = new DigitalInput(ShooterConstants.LimitSwitchPort);
    private boolean isBallPrimed = false;
    private boolean onTarget = false;
    
    public void shooterSubsystem() {

    }

    @Override
    public void periodic() {
    
    
}

public void primeBall(){
    m_conveyorMiddle.setInverted(false);
   
    if(!limitSwitch.get()){
        m_conveyorMiddle.set(0);
        isBallPrimed = true;
        return;
      } else {
          m_conveyorMiddle.set(ShooterConstants.conveyorlowPower);
          if(!limitSwitch.get()){
            m_conveyorMiddle.set(0);
            isBallPrimed = true;
            return;
          } else {
            isBallPrimed = false;
          }
      }
    }
  
    public void m_conveyorMiddleOn (){
      m_conveyorMiddle.set(ShooterConstants.conveyorlowPower);
    }
    
    }





