package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    //m_conveyorMiddle.setInverted(false);
    m_intakeFront.setInverted(true);
    m_intakeFront.set(ShooterConstants.intakePower);
    //m_conveyorMiddle.set(ShooterConstants.conveyorLowPower);
    
    /*
    if(limitSwitch.get()){
      m_conveyorMiddle.set(0);
      isBallPrimed = true;
      Return;
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

 

    // Method to to reverse Intake and eject balls
    public void ejectBall(){
     // m_conveyorMiddle.setInverted(true);
      m_intakeFront.setInverted(false);
      m_intakeFront.set(ShooterConstants.intakePower);
     // m_conveyorMiddle.set(ShooterConstants.conveyorHighPower);
    }

    
    public void intakeOff(){
      m_intakeFront.set(0);
      m_conveyorMiddle.set(0);
    }


    public void m_conveyorMiddleOn (){
      m_conveyorMiddle.set(ShooterConstants.conveyorLowPower);
      m_conveyorMiddle.setInverted(false);
    }

    public void m_conveyorMiddleOff (){
      m_conveyorMiddle.set(0);
    }

      ///////////////    Shooter Command   //////////////

  public void shooterOn (double speedOfShooter){
    m_shooterEnd.setInverted(true);
    m_conveyorMiddle.setInverted(false);
    
    m_shooterEnd.set(speedOfShooter);
    m_conveyorMiddle.set(ShooterConstants.conveyorHighPower);

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

  ////////////  Turn off Shooter Motor and Conveyer Motor ////////////////
  public void shooterOff(){
    m_shooterEnd.set(0);
    m_conveyorMiddle.set(0);
  }

}







