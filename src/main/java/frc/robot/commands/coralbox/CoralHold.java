// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralbox;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CoralBox;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralHold extends Command {
  /** Creates a new CoralHold. */
  private CoralBox m_CoralBox;
  public CoralHold(CoralBox subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_CoralBox = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("start cmd: " + getName());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!(m_CoralBox.getDistance() < 145)){
      m_CoralBox.spin(.6);
    }else{
    m_CoralBox.spin(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log(interrupted ? "interrupt cmd: " + getName() : "end cmd: " + getName());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
