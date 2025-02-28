// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralbox;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralBoxConstants;
import frc.robot.subsystems.CoralBox;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralOut extends Command {

  private CoralBox m_CoralBox;
  private DoubleSupplier m_speed;

  /** Creates a new CoralOut. */
  public CoralOut(CoralBox subsystem, DoubleSupplier speed) {
    m_CoralBox = subsystem;
    m_speed = speed;
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

    m_CoralBox.spin(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_CoralBox.spin(0); 
   DataLogManager.log(interrupted ? "interrupt cmd: " + getName() : "end cmd: " + getName());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
