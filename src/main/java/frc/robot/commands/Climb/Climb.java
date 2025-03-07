// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  /** Creates a new Climb. */
  private Climb m_rightClimb;
  private Climb m_leftClimb;
  private DoubleSupplier m_speed;

  public Climb(Climb right, Climb left, DoubleSupplier speed) {
    m_rightClimb = right;
    m_leftClimb = left;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     m_leftClimb.spin(.25);
     m_rightClimb.spin(.25);
       }
     
       private void spin(double speed) {
         // TODO Auto-generated method stub
         throw new UnsupportedOperationException("Unimplemented method 'spin'");
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
