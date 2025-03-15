// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoScoring;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorPIDSetpoints;
import frc.robot.commands.coralbox.CoralOut;
import frc.robot.commands.elevator.ElevatorToSetpoint;
import frc.robot.Constants.AutoScoreConstants.Side;
import frc.robot.subsystems.CoralBox;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopDriveToNearestReef extends InstantCommand {

  DriveSubsystem m_DriveSubsystem;
  Elevator m_elevator; 
  CoralBox m_IntakeSubsystem; 
  double setPoint; 
  Supplier<Translation2d> offset;
  Supplier<Side> side;

  public TeleopDriveToNearestReef(
    DriveSubsystem m_DriveSubsystem,
    Elevator m_elevator,
    CoralBox m_IntakeSubsystem, 
    double setPoint, 
    Supplier<Translation2d> offset,
    Supplier<Side> side
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_DriveSubsystem = m_DriveSubsystem;
    this.m_elevator = m_elevator;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.setPoint = setPoint;
    this.offset = offset;
    this.side = side;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
     if (side.get() != null) {

      Command commandToRun;

      Command MoveArmAndElevator;
      if(setPoint == ElevatorPIDSetpoints.Base) {
        MoveArmAndElevator = new ElevatorToSetpoint(ElevatorPIDSetpoints.L1, m_elevator).asProxy();
      } else if(setPoint == ElevatorPIDSetpoints.L2) {
          MoveArmAndElevator = new ElevatorToSetpoint(ElevatorPIDSetpoints.L2, m_elevator).asProxy(); 
      } else if ((setPoint == ElevatorPIDSetpoints.L3)) {
        MoveArmAndElevator = new ElevatorToSetpoint(ElevatorPIDSetpoints.L3, m_elevator).asProxy(); 
      } else if(setPoint == ElevatorPIDSetpoints.L4) {
        MoveArmAndElevator = new ElevatorToSetpoint(ElevatorPIDSetpoints.L4, m_elevator).asProxy(); 
      }else {
        MoveArmAndElevator = new PrintCommand("your auto score command is broken cause this is an invalid setpoint");
      }
      commandToRun = m_DriveSubsystem.driveToFirstAutoScorePose(side.get()).andThen( 
      new ParallelDeadlineGroup(
        m_DriveSubsystem.driveToSecondAutoScorePose(side.get(), offset.get()), 
        MoveArmAndElevator)).andThen(
      new ParallelDeadlineGroup( new WaitCommand(0.5), new CoralOut(m_IntakeSubsystem, ()-> 0.5))
      );

      System.out.println("should be running");
      // commandToRun.addRequirements(m_ArmSubsystem, m_DriveSubsystem, m_ElevatorSubsystem, m_IntakeSubsystem);
      commandToRun.schedule();
    }
    else {
      System.out.println("hey you did not pick a side :(");
    }
  }
}
