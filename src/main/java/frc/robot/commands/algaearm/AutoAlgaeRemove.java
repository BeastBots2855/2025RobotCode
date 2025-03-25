// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algaearm;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.ElevatorPIDSetpoints;
import frc.robot.commands.coralbox.CoralHold;
import frc.robot.commands.coralbox.CoralJuggle;
import frc.robot.commands.coralbox.CoralOut;
import frc.robot.commands.elevator.ElevatorToSetpoint;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralBox;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlgaeRemove extends SequentialCommandGroup {
  /** Creates a new AutoAlgaeRemove. */

  public AutoAlgaeRemove(Elevator elevatorSubsystem, CoralBox coralSubsystem, AlgaeArm algaeSubsystem, double elevatorSetpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ElevatorToSetpoint(elevatorSetpoint, elevatorSubsystem),
     //new GoToSetpoint(algaeSubsystem, AlgaeArmConstants.kUp),
     
    // new RunCommand(()->coralSubsystem.spin(1.0), coralSubsystem),
     new RunCommand(()->algaeSubsystem.move(-.5)).withTimeout(1.5),
     new RunCommand(()->coralSubsystem.spin(-1)).withTimeout(2),
      //new GoToSetpoint(algaeSubsystem, AlgaeArmConstants.kDown),
      new RunCommand(()->algaeSubsystem.move(1.0)).withTimeout(.5),
      new ElevatorToSetpoint(ElevatorPIDSetpoints.Base, elevatorSubsystem)
     );
  }
}
