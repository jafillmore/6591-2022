package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
    public final  CANSparkMax m_intakeFront = new CANSparkMax(ShooterConstants.intake, MotorType.kBrushed);
    private final CANSparkMax m_conveyorMiddle = new CANSparkMax(ShooterConstants.conveyor, MotorType.kBrushed);
    private final CANSparkMax m_shooterEnd = new CANSparkMax(ShooterConstants.shooter, MotorType.kBrushless);    
  
    private final RelativeEncoder m_shooterEndEncoder = m_shooterEnd.getEncoder();
     private SparkMaxPIDController m_shooterPID = m_shooterEnd.getPIDController();

     
    
     private boolean isBallPrimed = false;
    private boolean onTarget = false;
    
    public void shooterSubsystem() {


    }

    @Override
    public void periodic() {
    
    
}

  public void primeBall(){
    m_conveyorMiddle.setInverted(false);
    m_intakeFront.setInverted(true);
    m_intakeFront.set(ShooterConstants.intakePower);
    m_conveyorMiddle.set(ShooterConstants.conveyorLowPower);
    
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
      m_conveyorMiddle.setInverted(true);
      m_intakeFront.setInverted(false);
      m_intakeFront.set(ShooterConstants.intakePower);
      m_conveyorMiddle.set(ShooterConstants.conveyorHighPower);
    }

    
    public void intakeOff(){
      m_intakeFront.set(0);
      m_conveyorMiddle.set(0);
    }


    public void m_conveyorMiddleOn (){
      m_conveyorMiddle.set(ShooterConstants.conveyorLowPower);
    }



      ///////////////    Shooter Command   //////////////

  public void shooterOn (double speedOfShooter){
    m_shooterEnd.setInverted(true);
    m_conveyorMiddle.setInverted(false);
    
    m_shooterPID.setP(ShooterConstants.kP);
     m_shooterPID.setI(ShooterConstants.kI);
     m_shooterPID.setD(ShooterConstants.kD);
     m_shooterPID.setIZone(ShooterConstants.kIz);
     m_shooterPID.setFF(ShooterConstants.kFF);
     m_shooterPID.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
    
    m_shooterPID.setReference(speedOfShooter, CANSparkMax.ControlType.kVelocity);
    m_conveyorMiddle.set(ShooterConstants.conveyorHighPower);
    

     SmartDashboard.putNumber("Actual Motor RPM", (m_shooterEndEncoder.getVelocity()));
     SmartDashboard.putNumber("Target Motor RPM", (speedOfShooter));
    
   

     if(m_shooterEndEncoder.getVelocity() >= (speedOfShooter -ShooterConstants.AllowableSpeedError)){
        m_conveyorMiddle.set(ShooterConstants.conveyorHighPower);
      } else if(m_shooterEndEncoder.getVelocity() <= speedOfShooter/3-ShooterConstants.AllowableSpeedError) {
        m_conveyorMiddle.set(0);
      }
    } 
  

  ////////////  Turn off Shooter Motor and Conveyer Motor ////////////////
  public void shooterOff(){
    m_shooterEnd.set(0);
    m_conveyorMiddle.set(0);
  }

}





