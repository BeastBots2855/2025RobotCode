// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevator extends Command {
  /** Creates a new MoveElevator. */
  private Elevator m_elevator;
  private DoubleSupplier speedSupplier;

  public MoveElevator(Elevator subsystem, DoubleSupplier speedSupplier ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = subsystem;
    this.speedSupplier = speedSupplier;
    addRequirements(subsystem);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.PIDOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_elevator.move(speedSupplier.getAsDouble() + ElevatorConstants.feedForward);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.PIDOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
