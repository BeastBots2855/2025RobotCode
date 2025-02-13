// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralbox;

import java.util.function.DoubleSupplier;
import frc.robot.Constants.CoralBoxConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralBox;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralJuggle extends Command {
  /** Creates a new CoralJuggle. */
  private CoralBox m_CoralBox; 
  private int rotsForward;
  private int rotsBackward;
  private boolean forward;

  public CoralJuggle(CoralBox subsystem, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_CoralBox = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotsForward = 0;
    rotsBackward = 0;
    forward = true;
  }

  private void spinForward(){
    m_CoralBox.spin(CoralBoxConstants.kSpeed);
    rotsForward++;
  }

  private void spinBackward(){
    m_CoralBox.spin(CoralBoxConstants.kSpeed * -1);
    rotsBackward++;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(forward){
    while (rotsForward <= 2000){
      spinForward();
    }
    }else if(!forward){
      while (rotsBackward <= 500){
        spinBackward();
      }
    } 
    if(rotsForward >= 2000){
      forward = false;
    } else if(rotsBackward >= 500){
      forward = true;
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
