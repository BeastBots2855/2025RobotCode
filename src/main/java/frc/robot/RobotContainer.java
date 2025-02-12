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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorPIDSetpoints;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.coralbox.CoralOut;
import frc.robot.commands.elevator.CalibrateElevator;
import frc.robot.commands.elevator.ElevatorToSetpoint;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.subsystems.CoralBox;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
  XboxController m_fightstick = new XboxController(OIConstants.kFightStickPort);
  ShuffleboardTab tab = Shuffleboard.getTab("main tab");
  
  private final TalonFXS m_left = new TalonFXS(20);
  private final TalonFXS m_right = new TalonFXS(21);
  private final DigitalInput m_elevatorLimitSwitch = new DigitalInput(0);
  private final Elevator m_elevator = new Elevator(m_left, m_right, m_elevatorLimitSwitch);
  private final SparkMax m_boxMotor = new SparkMax(10, MotorType.kBrushless);
  private final CoralBox m_CoralBox = new CoralBox(m_boxMotor);

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    
    m_operatorController.axisGreaterThan(3, .05).whileTrue(new CoralOut(m_CoralBox, ()->m_operatorController.getRightTriggerAxis()));
    m_operatorController.axisGreaterThan(2, .05).whileTrue(new CoralOut(m_CoralBox, ()->m_operatorController.getLeftTriggerAxis() * -1));

    //m_operatorController.button(2).onTrue(new CalibrateElevator(m_elevator));
    


   // m_operatorController.button(3).onTrue(new RunCommand(()->m_elevator.setSetpoint(ElevatorPIDSetpoints.L1), m_elevator));
   // m_operatorController.button(1).onTrue(new RunCommand(()->m_elevator.setSetpoint(ElevatorPIDSetpoints.L2), m_elevator)); 
   // m_operatorController.button(4).onTrue(new RunCommand(()->m_elevator.(ElevatorPIDSetpoints.L3), m_elevator));
   // m_operatorController.button().onTrue(new RunCommand(()->m_elevator.setSetpoint(ElevatorPIDSetpoints.L4), m_elevator));

    m_operatorController.button(3).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L3, m_elevator));
    m_operatorController.button(2).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L1, m_elevator));
    m_operatorController.button(1).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L2, m_elevator));
    m_operatorController.button(4).onTrue(new ElevatorToSetpoint(ElevatorPIDSetpoints.L4, m_elevator));



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }



    public Elevator getElevator(){
        return m_elevator;
    }
}
