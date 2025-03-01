// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalibrateElevator extends Command {
  /** Creates a new CalibrateElevator. */
  private Elevator m_elevator;
  public CalibrateElevator(Elevator subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.PIDOff();
    DataLogManager.log("start cmd: " + getName());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_elevator.move(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
    m_elevator.resetEncoders();
    DataLogManager.log(interrupted ? "interrupt cmd: " + getName() : "end cmd: " + getName());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.isLimitSwitchPressed();
  }
}
