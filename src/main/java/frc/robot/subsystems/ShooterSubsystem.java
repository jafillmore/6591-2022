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

public class ShooterSubsystem extends SubsystemBase {
    public final  CANSparkMax m_intakeFront = new CANSparkMax(ShooterConstants.intake, MotorType.kBrushed);
    private final CANSparkMax m_conveyorMiddle = new CANSparkMax(ShooterConstants.conveyor, MotorType.kBrushed);
    private final CANSparkMax m_shooterEnd = new CANSparkMax(ShooterConstants.shooter, MotorType.kBrushless);    
  
    public void test() {

    }

    @Override
    public void periodic() {
    
    }}
    if(!limitSwitch.get()){
        primeMotor.set(0);
        isBallPrimed = true;
        return;
      } else {
          primeMotor.set(ShooterConst.primeMotorPrimeSpeed);
          if(!limitSwitch.get()){
            primeMotor.set(0);
            isBallPrimed = true;
            return;
          } else {
            isBallPrimed = false;
          }
      }
    }
  
    public void primeMotorOn (){
      primeMotor.set(ShooterConst.primeMotorPrimeSpeed);
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //Both of the methods below were replaced by one method that takes in a shooterSpeed as a parameter
  
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    public void shootOn(){
  <<<<<<< HEAD
      shooterMotor.set(-shooterSpeed);
    
      SmartDashboard.putNumber("Velocity for Encoder", encoder.getVelocity());
  =======
      shooterMotor.setInverted(false);
      
      primeMotor.setInverted(false);
      PID.setReference(shooterSpeed, ControlType.kVelocity);
      SmartDashboard.putNumber("Velocity from Encoder", encoder.getVelocity());
      SmartDashboard.putNumber("ShooterSpeed from ShootOn Command", shooterSpeed);
      
      if(!isBallPrimed){
        primeBall();
      } else {
       
        if(encoder.getVelocity() >= (shooterSpeed/3 -500)){
          primeMotor.set(ShooterConst.primeMotorShootSpeed);
        } else if(encoder.getVelocity() <= shooterSpeed/3-500) {
          primeMotor.set(0);
        }
      }
    }
    public void fastShoot(){
      shooterMotor.setInverted(false);
      
      primeMotor.setInverted(false);
}}





