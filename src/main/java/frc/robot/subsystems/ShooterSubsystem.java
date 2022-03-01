package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
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
    m_intakeFront.set(ShooterConstants.intakePower);
    m_conveyorMiddle.set(ShooterConstants.conveyorlowPower);
   
    /*
      if(limitSwitch.get()){
          m_conveyorMiddle.set(0);
          isBallPrimed = true;
          return;
        } else {
            m_conveyorMiddle.set(ShooterConstants.conveyorlowPower);
            if(!limitSwitch.get()){
              m_conveyorMiddle.set(0);
              isBallPrimed = false;
              return;
            } 

          }
    */
    }

    
    public void intakeOff(){
      m_intakeFront.set(0);
      m_conveyorMiddle.set(0);
    }


    public void m_conveyorMiddleOn (){
      m_conveyorMiddle.set(ShooterConstants.conveyorlowPower);
    }



      ////////////////////////////////////     New Shooter Command (if it doesn't work it is Jade's fault...)  //////////////
  //If this method does not work, uncomment the methods above and change the method that the button press calls in RobotContainer
  public void shooterOn (double speedOfShooter){
    m_shooterEnd.setInverted(true);
    m_conveyorMiddle.setInverted(false);
    
    m_shooterEnd.set(speedOfShooter);
    m_conveyorMiddle.set(ShooterConstants.conveyorhighPower);

   // PID.setReference(speedOfShooter, ControlType.kVelocity);

    // SmartDashboard.putNumber("Actual Motor RPM", (encoder.getVelocity()));
    //SmartDashboard.putNumber("Target Motor RPM", (speedOfShooter/3));
    
    /*if(!isBallPrimed){
      primeBall();
    } else {
*/
    /*  if(encoder.getVelocity() >= (speedOfShooter/3 -PIDConst.AllowableSpeedError)){
        primeMotor.set(ShooterConst.primeMotorShootSpeed);
      } else if(encoder.getVelocity() <= speedOfShooter/3-PIDConst.AllowableSpeedError) {
        primeMotor.set(0);
      }
    //} */
  }

  ////////////  Turn off Shooter Motor and Priming Motor ////////////////
  public void shooterOff(){
    m_shooterEnd.set(0);
    m_conveyorMiddle.set(0);
  }

}





