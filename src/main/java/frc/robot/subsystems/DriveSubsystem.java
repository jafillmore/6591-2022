// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  //Define the motor contollers
  private final CANSparkMax m_frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rearRight = new CANSparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);

  //Assign motors to Mechanum Drive
  private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

  // Define the Encoders
  private final RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
  private final RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private final RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  private final RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  // The gyro sensor
  // Updated to Kauai Labs NavX-MPX - see details at:  https://www.kauailabs.com/navx-mxp/
  private final AHRS ahrs = new AHRS();

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(DriveConstants.kDriveKinematics, ahrs.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    m_frontLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rearLeftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    m_rearRightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);
   
    m_drive.setDeadband(0.06);
    
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Shooter Motor Temp", m_frontLeft.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Motor Temp", m_frontRight.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Motor Temp", m_rearLeft.getMotorTemperature());
    SmartDashboard.putNumber("Shooter Motor Temp", m_rearRight.getMotorTemperature());

    // Update the odometry in the periodic block
    m_odometry.update(
        ahrs.getRotation2d(),
        new MecanumDriveWheelSpeeds(
            m_frontLeftEncoder.getVelocity(),
            m_rearLeftEncoder.getVelocity(),
            m_frontRightEncoder.getVelocity(),
            m_rearRightEncoder.getVelocity()));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, ahrs.getRotation2d());
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(Math.abs(ySpeed)*ySpeed, Math.abs(xSpeed)*xSpeed, rot, -ahrs.getAngle());
    } else {
      m_drive.driveCartesian(Math.abs(ySpeed)*ySpeed, Math.abs(xSpeed)*xSpeed, Math.abs(rot)*rot);
    }
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeftEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public RelativeEncoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public RelativeEncoder getRearLeftEncoder() {
    return m_rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public RelativeEncoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public RelativeEncoder getRearRightEncoder() {
    return m_rearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getVelocity(),
        m_rearLeftEncoder.getVelocity(),
        m_frontRightEncoder.getVelocity(),
        m_rearRightEncoder.getVelocity());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    ahrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -ahrs.getRate();
  }


  public void moveDangIt() {
   m_drive.driveCartesian(1.0,0,0);
  }
  
  
   public void stopDangIt() {
    m_drive.driveCartesian(0,0,0);
   }


}
