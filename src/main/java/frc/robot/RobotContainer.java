// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ExampleAutonomous;
import frc.robot.commands.ClimberReset;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.ThreeBall;
import frc.robot.commands.TwoBall;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.IntakeStorage;
import frc.robot.commands.OneBallFender;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterMax;
import frc.robot.commands.ShooterSlow;
import frc.robot.commands.ShooterStop;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Climber m_Climber = new Climber();
  private final Intake m_Intake = new Intake();
  private final Shooter m_Shooter = new Shooter();
  private final Conveyor m_Conveyor = new Conveyor();
  private final SlewRateLimiter m_SlewRateLimiter = new SlewRateLimiter(1.25);

  private static RobotContainer instance;


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
    // Configure the button bindings
    configureButtonBindings();
    RamseteFactory.setRobotDrive(m_robotDrive);

    instance = this;

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.closedLoopDrive(
                    m_SlewRateLimiter.calculate((m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis()) * Constants.DriveConstants.kMaxSpeed), m_driverController.getRightX() * Constants.DriveConstants.kMaxAngularSpeed),
            m_robotDrive));
  }

  public static RobotContainer getInstance() {
      return instance;
  }
  public Intake getIntake() {
      return m_Intake;
  }
  public Conveyor getConveyor() {
    return m_Conveyor;
  }
  public Shooter getShooter() {
      return m_Shooter;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    //new JoystickButton(m_driverController, Button.kRightBumper.value)
    //    .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
    //    .whenReleased(() -> m_robotDrive.setMaxOutput(1));

    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileHeld(new ClimberDown(m_Climber));

    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileHeld(new ClimberUp(m_Climber));

    new JoystickButton(m_operatorController, XboxController.Button.kBack.value)
        .whileHeld(new ClimberReset(m_Climber));

    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whenPressed(new IntakeDown(m_Intake, m_Conveyor));

    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whenPressed(new IntakeStorage(m_Intake, m_Conveyor));
    
    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .whileHeld(new IntakeReverse(m_Intake, m_Conveyor));

    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .whenPressed(new ShooterMax(m_Shooter));

    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .whenPressed(new ShooterSlow(m_Shooter));
    
    new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
        .whenPressed(new ShooterStop(m_Shooter));
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whenHeld(new Shoot(m_Conveyor, m_Intake));


    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new ThreeBall();
  }
}
