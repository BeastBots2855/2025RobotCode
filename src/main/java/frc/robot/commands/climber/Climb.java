// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  /** Creates a new Climb. */
  private Climber m_climb;
  private DoubleSupplier m_speed;

  public Climb(DoubleSupplier speed, Climber climb) {
    m_speed = speed;
    m_climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
  }
 
  
  
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    m_climb.spin(m_speed.getAsDouble());
  }
     
      
     
     
       // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.spin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
