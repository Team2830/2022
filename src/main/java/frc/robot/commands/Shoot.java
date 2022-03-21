// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class Shoot extends CommandBase {
  private Conveyor m_Conveyor;
  private Intake m_Intake;
  /** Creates a new Shoot. */
  public Shoot(Conveyor conveyor, Intake intake) {
    m_Conveyor = conveyor;
    m_Intake = intake; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Conveyor, m_Intake);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Conveyor.ConveyorUp();
    m_Intake.intakeIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Conveyor.ConveyorStop();
    m_Intake.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
