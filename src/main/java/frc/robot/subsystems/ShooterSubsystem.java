package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    public final  CANSparkMax m_intakeFront = new CANSparkMax(ShooterConstants.intake, MotorType.kBrushed);
    private final CANSparkMax m_conveyorMiddle = new CANSparkMax(ShooterConstants.conveyor, MotorType.kBrushed);
    private final CANSparkMax m_shooterEnd = new CANSparkMax(ShooterConstants.shooter, MotorType.kBrushless);    
  
    private final RelativeEncoder m_shooterEndEncoder = m_shooterEnd.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    private SparkMaxPIDController m_shooterPID = m_shooterEnd.getPIDController();
    
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher =new ColorMatch();
    Color detectedColor = m_colorSensor.getColor();
    
    private String colorString;
    private boolean isBallPrimed = false;
    private boolean onTarget = false;

    // Color Sensor Targets  
    private final Color kBlueTarget = new Color(0.143, 0.311, 0.429);
    private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    private final Color kRedTarget = new Color(0.431, 0.321, 0.114);
    private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);


  
    
  
    public void shooterSubsystem() {

    }

    @Override
    public void periodic() {
      
      m_colorMatcher.addColorMatch(kBlueTarget);
      m_colorMatcher.addColorMatch(kGreenTarget);
      m_colorMatcher.addColorMatch(kRedTarget);
      m_colorMatcher.addColorMatch(kYellowTarget);

      Color detectedColor = m_colorSensor.getColor();

      /**
       * Run the color match algorithm on our detected color
       */
   
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
  
      if (match.color == kBlueTarget) {
        colorString = "Blue";
      } else if (match.color == kRedTarget) {
        colorString = "Red";
      } else if (match.color == kGreenTarget) {
        colorString = "Green";
      } else if (match.color == kYellowTarget) {
        colorString = "Yellow";
      } else {
        colorString = "Unknown";
      }
     
      SmartDashboard.putNumber("Actual Motor RPM", m_shooterEndEncoder.getVelocity());
      SmartDashboard.putNumber("Shooter Motor Temp", m_shooterEnd.getMotorTemperature());
      //SmartDashboard.putNumber("Red", detectedColor.red);
      //SmartDashboard.putNumber("Green", detectedColor.green);
      //SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Range", m_colorSensor.getIR());
      SmartDashboard.putString("Detected Color", colorString);
      SmartDashboard.putNumber("Confidence", match.confidence);
    }

  public void primeBall(){
    ColorMatchResult match2 = m_colorMatcher.matchClosestColor(detectedColor);
    m_conveyorMiddle.setInverted(false);
    m_intakeFront.setInverted(true);
    m_intakeFront.set(ShooterConstants.intakePower);
    
    if (colorString == "Red" || colorString == "Blue"){
      m_conveyorMiddle.set(0);
    } else {
      m_conveyorMiddle.set(ShooterConstants.conveyorLowPower);
    }

  }

  // Method to to reverse Intake and eject balls
  public void ejectBall(){
    m_conveyorMiddle.setInverted(true);
    m_intakeFront.setInverted(false);
    m_intakeFront.set(ShooterConstants.intakePower);
    m_conveyorMiddle.set(ShooterConstants.conveyorHighPower);
    m_shooterEnd.set(0.0);

  }

  // Method to turn off intake  
    public void intakeOff(){
      m_intakeFront.set(0);
      m_conveyorMiddle.set(0);
    }

  ///////////////    Method to turn on the conveyor (currently not used)   ////////////
    public void m_conveyorMiddleOn (){
      m_conveyorMiddle.set(ShooterConstants.conveyorHighPower);
    }

    public void m_conveyorMiddleOff (){
      m_conveyorMiddle.set(0);
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

  
    SmartDashboard.putNumber("Target Motor RPM", (speedOfShooter));
     
    if(m_shooterEndEncoder.getVelocity() >= (speedOfShooter -ShooterConstants.AllowableSpeedError)){
     // Timer.delay(0.25);
      m_conveyorMiddle.set(ShooterConstants.conveyorHighPower);
    } else if(m_shooterEndEncoder.getVelocity() <= speedOfShooter - ShooterConstants.AllowableSpeedError) {
      m_conveyorMiddle.set(0);
    }

  } 
  
  ////////////  Spin up shooter to a specific rpm
  public void shooterSpinUp (double speedOfShooter){
    m_shooterEnd.setInverted(true);
    m_conveyorMiddle.setInverted(false);
    
    m_shooterPID.setP(ShooterConstants.kP);
    m_shooterPID.setI(ShooterConstants.kI);
    m_shooterPID.setD(ShooterConstants.kD);
    m_shooterPID.setIZone(ShooterConstants.kIz);
    m_shooterPID.setFF(ShooterConstants.kFF);
    m_shooterPID.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);      
     
    m_conveyorMiddle.set(0);
    m_intakeFront.set(0);
    m_shooterPID.setReference(speedOfShooter, CANSparkMax.ControlType.kVelocity);

  }



  ////////////  Turn off Shooter Motor and Conveyer Motor ////////////////
  public void shooterOff(){
    //m_shooterEnd.set(0);
    
    /////////  set shooter speed to low speed  //////////
    m_shooterPID.setReference(ShooterConstants.shooterLowPower, CANSparkMax.ControlType.kVelocity);
    
    m_conveyorMiddle.set(0);
  }

}





