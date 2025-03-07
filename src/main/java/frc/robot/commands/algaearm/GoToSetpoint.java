// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algaearm;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.subsystems.AlgaeArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToSetpoint extends Command {
  private AlgaeArm m_AlgaeArm;
  private double m_setpoint;
  /** Creates a new go. */
  public GoToSetpoint(AlgaeArm subsystem, double setpoint) {
    addRequirements(subsystem);
    m_AlgaeArm = subsystem;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_AlgaeArm.setSetpoint(m_setpoint);
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
