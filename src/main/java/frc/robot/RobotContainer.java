// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.*;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ClimberConstants;
// ******** Not sure what this import is for....  import pabeles.concurrency.ConcurrencyOps.NewInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  // The driver's controller
  private Joystick m_leftStick = new Joystick(JoystickConstants.kLeftJoystickPort);
  private Joystick m_rightStick = new Joystick(JoystickConstants.kRightJoystickPort);
  private Joystick m_bBox = new Joystick(JoystickConstants.kButtonBoxPort);
  //private Joystick m_buttonBox = new Joystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    m_rightStick.getX(),
                    -m_leftStick.getY(),
                    m_rightStick.getZ(),
                    false),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_rightStick, JoystickConstants.kDriveSpeedLimiterButton)
        .whenPressed(() -> m_robotDrive.setMaxOutput(DriveConstants.kLowSpeedDrivePowerLimit))
        .whenReleased(() -> m_robotDrive.setMaxOutput(1));

    new JoystickButton(m_leftStick, JoystickConstants.kIntakeButton)
        .whenHeld(new RunCommand(() -> shooterSubsystem.primeBall()))
        .whenReleased(new RunCommand(() -> shooterSubsystem.intakeOff()));

    new JoystickButton(m_rightStick, JoystickConstants.kShootHighButton)
    .whenHeld(new InstantCommand(() -> shooterSubsystem.shooterOn(ShooterConstants.shooterHighPower)))
    .whenReleased(new InstantCommand(() -> shooterSubsystem.shooterOff()));
  

    new JoystickButton(m_rightStick, JoystickConstants.kShootLowButton)
    .whenHeld(new InstantCommand(() -> shooterSubsystem.shooterOn(ShooterConstants.shooterLowPower)))
    .whenReleased(new InstantCommand(() -> shooterSubsystem.shooterOff()));


    new JoystickButton(m_bBox, JoystickConstants.kLeftArmFowardButton)
    .whenHeld(new InstantCommand(() -> climberSubsystem.leftArmForward()))
    .whenReleased(new InstantCommand(() -> climberSubsystem.leftArmOff()));

    new JoystickButton(m_bBox, JoystickConstants.kRightArmForwardButton)
    .whenHeld(new InstantCommand(() -> climberSubsystem.rightArmForward()))
    .whenReleased(new InstantCommand(() -> climberSubsystem.rightArmOff()));

    new JoystickButton(m_bBox, JoystickConstants.kRightArmForwardButton)
    .whenHeld(new InstantCommand(() -> climberSubsystem.leftArmBack()))
    .whenReleased(new InstantCommand(() -> climberSubsystem.leftArmOff()));

    new JoystickButton(m_bBox, JoystickConstants.kRightArmBackButton)
    .whenHeld(new InstantCommand(() -> climberSubsystem.rightArmBack()))
    .whenReleased(new InstantCommand(() -> climberSubsystem.rightArmOff()));

/////// Insert new buttons above here!
}





  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    MecanumControllerCommand mecanumControllerCommand =
        new MecanumControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            DriveConstants.kFeedforward,
            DriveConstants.kDriveKinematics,

            // Position contollers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

            // Needed for normalizing wheel speeds
            AutoConstants.kMaxSpeedMetersPerSecond,

            // Velocity PID's
            new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
            new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
            new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
            new PIDController(DriveConstants.kPRearRightVel, 0, 0),
            m_robotDrive::getCurrentWheelSpeeds,
            m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
