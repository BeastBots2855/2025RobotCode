// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algaearm;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    setName("Algae Arm: " + getName() + " " + setpoint);  //so multiple instances can be distinguished
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_AlgaeArm.PIDOn();
    DataLogManager.log("start cmd: " + getName());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_AlgaeArm.setSetpoint(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log(interrupted ? "interrupt cmd: " + getName() : "end cmd: " + getName());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return new WaitCommand(.5).isFinished();
}
}
