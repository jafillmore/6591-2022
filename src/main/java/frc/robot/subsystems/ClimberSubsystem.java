package frc.robot.subsystems;


import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

public class ClimberSubsystem {
    

    public class DriveSubsystem extends SubsystemBase {
        private final CANSparkMax m_positionLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
        private final CANSparkMax m_climberLeft = new CANSparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
        private final CANSparkMax m_climberRight = new CANSparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
        private final CANSparkMax m_positionRight = new CANSparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);
      
      
}

}
