// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorPIDSetpoints;
import frc.robot.Constants.OIConstants;
import frc.robot.utilities.RGBColor;
import frc.robot.commands.LEDCommands.RAINBOWS;
import frc.robot.commands.coralbox.CoralHold;
import frc.robot.commands.coralbox.CoralJuggle;
import frc.robot.commands.coralbox.CoralOut;
import frc.robot.commands.elevator.CalibrateElevator;
import frc.robot.commands.elevator.ElevatorToSetpoint;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.subsystems.CoralBox;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.utilities.RGBColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import LEDS

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  CommandXboxController m_fightstick = new CommandXboxController(OIConstants.kFightStickPort);
  ShuffleboardTab tab = Shuffleboard.getTab("main tab");
  
  private final TalonFXS m_left = new TalonFXS(20);
  private final TalonFXS m_right = new TalonFXS(21);
  private final DigitalInput m_elevatorLimitSwitch = new DigitalInput(0);
  private final Elevator m_elevator = new Elevator(m_left, m_right, m_elevatorLimitSwitch);
  private final SparkMax m_boxMotor = new SparkMax(10, MotorType.kBrushless);
  private final CoralBox m_CoralBox = new CoralBox(m_boxMotor);
  private final LED m_ledString = new LED(0);
//   private final LED m_ledStringRight = new LED(1);

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //auto named commands
    NamedCommands.registerCommand("CoralHold", new CoralHold(m_CoralBox));
    NamedCommands.registerCommand("TestEvent", new PrintCommand("TestEvent"));
    NamedCommands.registerCommand("ElevatorToL4", new ElevatorToSetpoint(ElevatorPIDSetpoints.L4, m_elevator));
    NamedCommands.registerCommand("ElevatorToL3", new ElevatorToSetpoint(ElevatorPIDSetpoints.L3, m_elevator));
    NamedCommands.registerCommand("ElevatorToL2", new ElevatorToSetpoint(ElevatorPIDSetpoints.L2, m_elevator));
    NamedCommands.registerCommand("ElevatorToBase", new ElevatorToSetpoint(ElevatorPIDSetpoints.Base, m_elevator));
    NamedCommands.registerCommand("CoralOut", new CoralOut(m_CoralBox,()->m_CoralBox.getAutoCoralSpeed()));

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    





    tab.addDouble("elevator position", ()->m_elevator.getPos());
    tab.addDouble ("elevator output(left)", ()->m_elevator.getOutput());
    tab.addDouble("elevator Setpoint", ()->m_elevator.getTargetPos());
    tab.addDouble("distance sensor", ()->m_CoralBox.getDistance());
    SmartDashboard.putData(m_robotDrive);
    tab.addDouble("XPos", () -> m_robotDrive.getPose().getX());
    tab.addDouble("YPos", () -> m_robotDrive.getPose().getY());
    tab.addDouble("robot angle", ()->m_robotDrive.getHeading());
    tab.addBoolean("limitSwitch pressed", ()->m_elevator.isLimitSwitchPressed());
    //m_ledString.setColor(Constants.Colors.yellow);

    //CommandScheduler.getInstance().schedule(new RAINBOWS(m_ledString));
    // CommandScheduler.getInstance().schedule(new RAINBOWS(m_ledStringRight));
    //m_ledString.setDefaultCommand(new RunCommand(()->m_ledString.setColor(255,100,0)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    // m_operatorController.axisGreaterThan(1, .1).whileTrue(new MoveElevator(m_elevator, ()->m_operatorController.getLeftY() * -1));

    new Trigger(()-> Math.abs(m_operatorController.getLeftY()) > 0.1 ).whileTrue(new MoveElevator(m_elevator, ()->m_operatorController.getLeftY() * -1));
   
   /**
    * driver can slow robot to 25% output by pressing either trigger
    */
    new Trigger(()->m_driverController.getRightTriggerAxis() > .3).whileTrue(new RunCommand(
      () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY() * .25, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX() * .25, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX() * .25, OIConstants.kDriveDeadband),
          true),
      m_robotDrive));
    new Trigger(()->m_driverController.getLeftTriggerAxis() > .3).whileTrue(new RunCommand(() -> m_robotDrive.drive(
      -MathUtil.applyDeadband(m_driverController.getLeftY() * .25, OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(m_driverController.getLeftX() * .25, OIConstants.kDriveDeadband),
      -MathUtil.applyDeadband(m_driverController.getRightX() * .25, OIConstants.kDriveDeadband),
      true),
  m_robotDrive));
      
      new Trigger(()->m_elevator.isLimitSwitchPressed() == true).onTrue(new WaitCommand(.1).andThen(new InstantCommand(()->m_elevator.resetEncoders())));
    
    /**
     * slows drive to 10% when elevator is above L2
     */
    
      new Trigger(()->m_elevator.getPos() > ElevatorPIDSetpoints.L2).whileTrue(new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY() * .1, OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX() * .1, OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX() * .1, OIConstants.kDriveDeadband),
            true),
        m_robotDrive));



    
    m_operatorController.axisGreaterThan(3, .05).whileTrue(new CoralOut(m_CoralBox, ()->m_operatorController.getRightTriggerAxis()));
    m_operatorController.axisGreaterThan(2, .05).whileTrue(new CoralOut(m_CoralBox, ()->m_operatorController.getLeftTriggerAxis() * -1));
    m_fightstick.axisGreaterThan(2, 0.5).whileTrue(new CoralJuggle(m_CoralBox, ()->m_fightstick.getLeftTriggerAxis()));
    m_fightstick.axisGreaterThan(3, 0.5).whileTrue(new CoralOut(m_CoralBox, ()->m_fightstick.getRightTriggerAxis() * -0.5));


    //m_operatorController.button(2).onTrue(new CalibrateElevator(m_elevator));
    


   // m_operatorController.button(3).onTrue(new RunCommand(()->m_elevator.setSetpoint(ElevatorPIDSetpoints.L1), m_elevator));
   // m_operatorController.button(1).onTrue(new RunCommand(()->m_elevator.setSetpoint(ElevatorPIDSetpoints.L2), m_elevator)); 
   // m_operatorController.button(4).onTrue(new RunCommand(()->m_elevator.(ElevatorPIDSetpoints.L3), m_elevator));
   // m_operatorController.button().onTrue(new RunCommand(()->m_elevator.setSetpoint(ElevatorPIDSetpoints.L4), m_elevator));

    m_operatorController.button(3).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L3, m_elevator));
    m_operatorController.button(2).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L1, m_elevator));
    m_operatorController.button(1).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L2, m_elevator));
    m_operatorController.button(4).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L4, m_elevator));
    m_fightstick.button(9).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L1, m_elevator));
    m_fightstick.button(4).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L2, m_elevator));
    m_fightstick.button(1).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L3, m_elevator));
    m_fightstick.button(2).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L4, m_elevator));
    m_fightstick.button(5).onTrue(new CoralOut(m_CoralBox, ()-> 0.5));
    m_fightstick.button(
      3).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.Base, m_elevator).andThen(new RunCommand(()->m_elevator.PIDOff())));
    m_fightstick.button(10).onTrue(new InstantCommand(()->m_elevator.resetEncoders()));
    //fightstick intake to lightsensor button 6
    m_fightstick.button(6).onTrue(new CoralHold(m_CoralBox));
    new JoystickButton(m_driverController, 8).onTrue(new InstantCommand(()->m_robotDrive.zeroHeading()
    ));//new RunCommand(()->m_robotDrive.zeroHeading()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("LeftSideAuto");
    // return AutoBuilder.buildAuto("TestAuto");
    // return new PathPlannerAuto("TestAuto");
    // return new PrintCommand("yeah");
    // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

   
  }



    public Elevator getElevator(){
        return m_elevator;
    }

    public LED getLED(){
        return m_ledString;
    }

    // public LED getLEDRight(){
    //     return m_ledStringRight;
    // }
}
