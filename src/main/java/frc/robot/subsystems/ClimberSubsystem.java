package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;


public class ClimberSubsystem extends SubsystemBase {
    // Define Climb Motor Controllers    
    private final CANSparkMax m_positionLeft = new CANSparkMax(ClimberConstants.kLeftArmPositionMotorPort, MotorType.kBrushed);
    private final CANSparkMax m_climberLeft = new CANSparkMax(ClimberConstants.kLeftWinchMotorPort, MotorType.kBrushed);
    private final CANSparkMax m_climberRight = new CANSparkMax(ClimberConstants.kRightWinchMotorPort, MotorType.kBrushed);
    private final CANSparkMax m_positionRight = new CANSparkMax(ClimberConstants.kRightArmPositionMotorPort, MotorType.kBrushed);
    
    // Define arm position encoders
    // *********** UPDATE NEEDED **********************
    // ******** Need to integrate the encoders used on the arm positioning ***********
    // ******** If we are using Seat Motors Details are here : https://wpilib.screenstepslive.com/s/4485/m/63630/l/679357-bosch-seat-motor
    
    //Define winch motor encoders
    // ******** External Quadurature Encoders will plug into the encoder port on the spark max
    // ******** See here for Spark Max "alternate encoder" info: https://docs.revrobotics.com/sparkmax/operating-modes/alternate-encoder-mode#connecting-an-alternate-encoder



    public ClimberSubsystem() { 
      
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    // ************************************************
    // Method to extend arms 
    // *********** UPDATE NEEDED **********************
    // Need to add a method using PID control to extend the arms to a target position
    public void extendArms(){
      m_climberLeft.setInverted(false);
      m_climberRight.setInverted(true);
      m_climberLeft.set(-ClimberConstants.winchPower);
      m_climberRight.set(ClimberConstants.winchPower);
    }
    // ************************************************
    // Method to retract arms 
    // *********** UPDATE NEEDED **********************
    // Need to add a method using PID control to retract the arms to a target position
    public void retractArms(){
      m_climberLeft.setInverted(true);
      m_climberRight.setInverted(false);
      m_climberLeft.set(-ClimberConstants.winchPower);
      m_climberRight.set(ClimberConstants.winchPower);
    }
   
    //  Turn off winch motors
    public void winchOff(){
      m_climberLeft.set(0);
      m_climberRight.set(0);
    }


    // ************************************************
    // Method to Rotate arms 
    // *********** UPDATE NEEDED **********************
    // Need to aadd a method using PID control to Rotate the arms to a target position
    
    // Temporary method using button box to position arms
    public void leftArmForward(){
      m_positionLeft.set(-ClimberConstants.rotatePower);
    }

    public void rightArmForward(){
      m_positionRight.set(ClimberConstants.rotatePower);
    }

    public void leftArmBack(){
      m_positionLeft.set(ClimberConstants.rotatePower);
    }

    public void rightArmBack(){
      m_positionRight.set(-ClimberConstants.rotatePower);
    }

    public void leftArmOff(){
      m_positionLeft.set(0);
    }

    public void rightArmOff(){
      m_positionRight.set(0);
    }

  



    // ************************************************
    // Method to automate climb
    // *********** UPDATE NEEDED **********************
    // Need to add a method to combine the above to automate the climbing process


}
