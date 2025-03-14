// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.coralbox;

import java.util.function.DoubleSupplier;
import frc.robot.Constants.CoralBoxConstants;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralBox;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralJuggle extends Command {
  /** Creates a new CoralJuggle. */
  private CoralBox m_CoralBox; 
  private Timer m_timer;
  
  private boolean forward;

  public CoralJuggle(CoralBox subsystem, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_CoralBox = subsystem;
    addRequirements(subsystem);
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    forward = true;
    DataLogManager.log("start cmd: " + getName());
  }

  private void spinForward(){
    m_CoralBox.spin(CoralBoxConstants.kSpeed);
  
  }

  private void spinBackward(){
    m_CoralBox.spin(CoralBoxConstants.kSpeed * -1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer.get() <= 1){
      spinForward();
    }else if(m_timer.get() >= 1 && m_timer.get() <= 1.125){
      spinBackward();
    }else if(m_timer.get() > 1.125){
      spinBackward();
      m_timer.reset();
     
    }
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
