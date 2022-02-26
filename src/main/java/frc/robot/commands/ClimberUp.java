// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberUp extends CommandBase {
   private Climber m_Climber;
  /** Creates a new ClimberUp. */
  public ClimberUp(Climber climber) {
    m_Climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if (m_Climber.getPosition() < Constants.ClimberConstants.kTopClimberMotorPosition){
    m_Climber.normalMovement();
  }
  else {
    m_Climber.stopMovement();
  }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climber.stopMovement();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Climber.getPosition() > Constants.ClimberConstants.kTopClimberMotorPosition;
  }
}
