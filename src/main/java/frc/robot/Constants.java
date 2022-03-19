// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class JoystickConstants{
    
    //Joystick USB Ports
    public static final int kLeftJoystickPort = 0;
    public static final int kRightJoystickPort = 1;
    public static final int kButtonBoxPort = 2;

    // Left Joystick Buttons
    public static final int kIntakeButton = 1;
    public static final int kReverseIntakeButton = 2;

    // Right Joystick Buttons
    public static final int kDriveSpeedLimiterButton = 1;
    public static final int kShootHighButton = 3;
    public static final int kShootLowButton = 4;

    // Button Box Buttons
    public static final int kLeftArmFowardButton = 11;
    public static final int kLeftArmBackButton = 12;
    public static final int kRightArmForwardButton = 13;
    public static final int kRightArmBackButton = 14;

  }
    
  public static final class ClimberConstants{
        
    // Motor CAN ID numbers 
    public static final int kLeftArmPositionMotorPort = 9;
    public static final int kLeftWinchMotorPort = 11;
    public static final int kRightWinchMotorPort = 10;
    public static final int kRightArmPositionMotorPort = 8;

    // Roborio Ports for arm position encoders
    // ********* UPDATE REQUIRED **********************
    // ********* Need to confirm Port IDs for seat motor encoders 

    // Power for arm rotation
    public static final double rotatePower = 0.25;

    // Constants for arm rotation PID Control
    // ********* UPDATE REQUIRED **********************
    // ********* Add these constants when the PID Control is added

    // Constants for winch motor PID Control
    // ********* UPDATE REQUIRED **********************
    // ********* Add these constants when the PID Control is added


  }


  public static final class ShooterConstants{
    
    // Intake and Shooter Motor CAN IDs
    public static final int intake = 5;
    public static final int conveyor = 6;
    public static final int shooter = 7;
    
    // Intake and Shooter Power Settings
    public static final double intakePower = 1; 
    public static final double conveyorLowPower = 0.6;
    public static final double conveyorHighPower = 1.0;
    public static final double shooterLowPower = 3000;
    public static final double shooterHighPower = 5000;

    public static final double kP = 6e-5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz =0; 
    public static final double kFF = 0.000015;
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1; 
    public static final double maxPRM = 5700;

    public static final double AllowableSpeedError = 300;

    // Conveyer Delay before Shooting
    public static final double conveyorDelay = 1.0;

    // Color Sensor Targets  
    public final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    public final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    public final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    public final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  }

  public static final class DriveConstants {
    public static final int kRearLeftMotorPort = 1;
    public static final int kRearRightMotorPort = 2;
    public static final int kFrontLeftMotorPort = 3;
    public static final int kFrontRightMotorPort = 4;

    public static final double kLowSpeedDrivePowerLimit = 0.5;
  
    public static final int[] kFrontLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRearLeftEncoderPorts = new int[] {2, 3};
    public static final int[] kFrontRightEncoderPorts = new int[] {4, 5};
    public static final int[] kRearRightEncoderPorts = new int[] {6, 7};

    public static final boolean kFrontLeftEncoderReversed = false;
    public static final boolean kRearLeftEncoderReversed = true;
    public static final boolean kFrontRightEncoderReversed = false;
    public static final boolean kRearRightEncoderReversed = true;

    public static final double kTrackWidth = 0.5;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.7;
    // Distance between centers of front and back wheels on robot

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.5;
    public static final double kPRearLeftVel = 0.5;
    public static final double kPFrontRightVel = 0.5;
    public static final double kPRearRightVel = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
