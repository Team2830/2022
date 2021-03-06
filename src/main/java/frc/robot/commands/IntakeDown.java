// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeDown extends CommandBase {
  private Intake m_Intake;
  private Conveyor m_Conveyor;
  /** Creates a new IntakeDown. */
  public IntakeDown(Intake intake, Conveyor conveyor) {
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
    m_Intake.intakedown();

    if ( ! m_Conveyor.getTopPhotoEye() & ! m_Conveyor.getBottomPhotoEye()){
      m_Intake.intakeIn();
      m_Conveyor.ConveyorUp();
    }
    if ( ! m_Conveyor.getTopPhotoEye() &  m_Conveyor.getBottomPhotoEye()){
      m_Intake.intakeIn();
      m_Conveyor.ConveyorUp();
    }
    if (m_Conveyor.getTopPhotoEye() & ! m_Conveyor.getBottomPhotoEye()){
      m_Intake.intakeIn();
      m_Conveyor.ConveyorStop();
    }
    if (m_Conveyor.getTopPhotoEye() & m_Conveyor.getBottomPhotoEye()){
      m_Intake.intakeStop();
      m_Conveyor.ConveyorStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
