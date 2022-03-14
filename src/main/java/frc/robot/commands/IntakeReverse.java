// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeReverse extends CommandBase {
  private Intake m_Intake;
  private Conveyor m_Conveyor;
  /** Creates a new IntakeReverse. */
  public IntakeReverse(Intake intake, Conveyor conveyor) {
    m_Intake = intake;
    m_Conveyor = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake, m_Conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.intakeOut();
    m_Conveyor.Conveyordown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.intakeStop();
    m_Conveyor.ConveyorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
