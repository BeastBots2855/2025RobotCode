// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
// Import only what is needed for the Elevator subsystem functionality.
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MusicTone;

/**
 * Elevator subsystem for controlling the elevator mechanism.
 *
 * This subsystem manages two TalonFXS motor controllers (left and right), utilizes a limit switch
 * for safety, and uses a PIDController to manage the elevator's position.
 */
public class Elevator extends SubsystemBase {

  // Motor controllers for the elevator's left and right sides.
  private final TalonFXS m_left;
  private final TalonFXS m_right;

  // Limit switch to detect the elevator's bottom (or top) position for safety.
  private final DigitalInput m_limitSwitch;

  // Flag to track if the current has exceeded the safe threshold.
  private boolean currentLimitExceeded;

  // The current desired setpoint for the elevator position.
  private double currDesiredSetpoint;

  // Flag indicating whether PID control is enabled.
  private boolean isPIDEnabled;

  /**
   * Constructs a new Elevator subsystem.
   *
   * @param left The left TalonFXS motor controller.
   * @param right The right TalonFXS motor controller.
   * @param limitswitch The digital input for the elevator limit switch.
   */
  public Elevator(TalonFXS left, TalonFXS right, DigitalInput limitswitch) {
    // Initialize motor controllers and limit switch.
    m_right = right;
    m_left = left;
    m_limitSwitch = limitswitch;

    // Set both motors to brake mode for safety when disabled.
    m_left.setNeutralMode(NeutralModeValue.Brake);
    m_right.setNeutralMode(NeutralModeValue.Brake);

    // --------------------------------------------------------------------------
    // Set up configurators and configurations for both elevator motor controllers.
    // --------------------------------------------------------------------------
    TalonFXSConfigurator leftTalonFXSConfigurator = m_left.getConfigurator();
    TalonFXSConfiguration leftTalonConfiguration = new TalonFXSConfiguration();
    TalonFXSConfigurator rightTalonFXSConfigurator = m_right.getConfigurator();
    TalonFXSConfiguration rightTalonConfiguration = new TalonFXSConfiguration();

    // --------------------------------------------------------------------------
    // Configure slot 0 parameters for the left elevator motor. This all exists
    // for the sake of using a trapezoidal motion profile for the elevator. The 
    // calculations are entirely handled on the motoro controller we just need to 
    // give it some values to be able to perform the calculations with. Also important 
    // to note that trapezoidal motion profile is just callecd motion magic by CTRE
    // so if you want to look at doucmentation you should go and look at the docs for
    // TalongFXS and look for motion magic.
    // Gains and feedforward constants:
    // kS: Static friction voltage,
    // kV: Velocity constant,
    // kA: Acceleration constant,
    // kP: Proportional gain,
    // kI: Integral gain,
    // kD: Derivative gain.
    // --------------------------------------------------------------------------
    var slot0ConfigsLeft = leftTalonConfiguration.Slot0;
    slot0ConfigsLeft.kS = 0.4; // Overcome static friction
    slot0ConfigsLeft.kV = 0.09; // Voltage per unit of velocity
    slot0ConfigsLeft.kA = 0.00; // Voltage per unit of acceleration (not used)
    slot0ConfigsLeft.kP = 0.55; // Proportional gain for position control
    slot0ConfigsLeft.kI = 0;   // Integral gain (disabled)
    slot0ConfigsLeft.kD = 0.000; // Derivative gain
    slot0ConfigsLeft.kG = 0.5;
    slot0ConfigsLeft.withGravityType(GravityTypeValue.Elevator_Static);

    // --------------------------------------------------------------------------
    // Configure Motion Magic parameters for the left elevator motor.
    // Motion Magic provides smooth motion profiling.
    // --------------------------------------------------------------------------
    var leftMotionMagicConfigs = leftTalonConfiguration.MotionMagic;
    leftMotionMagicConfigs.MotionMagicCruiseVelocity = 50; // Cruise velocity in rps
    leftMotionMagicConfigs.MotionMagicAcceleration = 200;    // Acceleration in rps/s
    leftMotionMagicConfigs.MotionMagicJerk = 1600;           // Jerk in rps/s/s

    // --------------------------------------------------------------------------
    // Configure slot 0 parameters for the right elevator motor (same as left).
    // --------------------------------------------------------------------------
    var slot0ConfigsRight = rightTalonConfiguration.Slot0;
    slot0ConfigsRight.kS = 0.4; //0.4
    slot0ConfigsRight.kV = 0.09; //0.213
    slot0ConfigsRight.kA = 0.00;
    slot0ConfigsRight.kP = 0.55;
    slot0ConfigsRight.kI = 0;
    slot0ConfigsRight.kD = 0.0;
    slot0ConfigsRight.kG = 0.5;
    slot0ConfigsRight.withGravityType(GravityTypeValue.Elevator_Static);

    // --------------------------------------------------------------------------
    // Configure Motion Magic parameters for the right elevator motor.
    // --------------------------------------------------------------------------
    var rightMotionMagicConfigs = rightTalonConfiguration.MotionMagic;
    rightMotionMagicConfigs.MotionMagicCruiseVelocity = 50; //64, 10, 50
    rightMotionMagicConfigs.MotionMagicAcceleration = 200; //640, 30, 100
    rightMotionMagicConfigs.MotionMagicJerk = 1600; //1600

    // --------------------------------------------------------------------------
    // Set the motor output inversion so that the motor rotates in the proper direction.
    // --------------------------------------------------------------------------
    rightTalonConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftTalonConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    
    // --------------------------------------------------------------------------
    // Apply the configurations to the motor controllers.
    // --------------------------------------------------------------------------
    leftTalonFXSConfigurator.apply(leftTalonConfiguration);
    rightTalonFXSConfigurator.apply(rightTalonConfiguration);


    stop();
  }

  /**
   * Resets the encoder positions of both elevator motors to zero.
   */
  public void resetEncoders() {
    m_right.setPosition(0);
    m_left.setPosition(0);
    System.out.println("reset encoders");
  }

  /**
  * Sets the voltage output for both elevator motors.
  *
  * @param voltage The voltage to apply to the motors.
  */
  public void setVoltage(double voltage) {
    m_right.setVoltage(voltage);
    m_left.setVoltage(voltage);
  }

  /**
  * Updates the desired setpoint for the elevator position. 
  *
  * @param setpoint The new setpoint for elevator positioning. The units are the number 
  * of rotations of the motor's output shaft.
  */
  public void setSetpoint(double setpoint) {
    currDesiredSetpoint = setpoint;
  }

  

  /**
  * Stops the elevator by setting motor outputs to zero.
  */
  public void stop() {
    m_left.set(0);
    m_right.set(0);

    isPIDEnabled = false;
  }

  public void hold(){
    m_left.set(ElevatorConstants.feedForward);
    m_right.set(ElevatorConstants.feedForward);
  }

  /**
  * Returns the output duty cycle of the left elevator motor.
  *
  * @return The duty cycle as a double value.
  */
  public double getOutput() {
    return m_left.getDutyCycle().getValueAsDouble();
  }

  /**
  * Disables PID control.
  */
  public void PIDOff() {
    isPIDEnabled = false;
  }

  /**
  * Enables PID control.
  */
  public void PIDOn() {
    isPIDEnabled = true;
  }

  /**
  * Moves the elevator at a specified speed.
  * 
  * If the limit switch is activated and the speed is negative (attempting to move down),
  * the function sets the speed to zero and stops the motors.
  * Otherwise, it scales the speed and applies it to both motors.
  *
  * @param speed The desired speed for the elevator.
  */
  public void move(double speed) {
    // Disable PID control while manually controlling the elevator.
    isPIDEnabled = false;

    if (m_limitSwitch.get() && speed < 0) {
        // Limit switch is triggered and attempting to move downward.
        speed = 0;
        m_left.set(0);
        m_right.set(0);
        System.out.println("at bottom");
    } else {
      
        // Scale speed by 0.25 and apply to both motors.

        m_left.set(speed *= .15);
        m_right.set(speed *= .15);
        System.out.println("running");
    }
  }

  /**
  * Returns the state of the limit switch.
  *
  * @return True if the limit switch is pressed, false otherwise.
  */
  public boolean isLimitSwitchPressed() {
    return !m_limitSwitch.get();
  }

  /**
  * Retrieves the current position of the elevator from the left motor's encoder.
  *
  * @return The current position as a double value.
  */
  public double getPos() {
    return m_left.getPosition().getValueAsDouble();
  }

  /**
  * Provides the current target setpoint for the elevator.
  *
  * @return The target position as a double value.
  */
  public double getTargetPos() {
    return currDesiredSetpoint;
  }



  @Override
  public void periodic() {
      // Check if the supply current for either motor exceeds 20 Amps and update the flag.
      currentLimitExceeded = ((m_left.getSupplyCurrent().getValueAsDouble() > 20) ||
                              (m_right.getSupplyCurrent().getValueAsDouble() > 20));
  
      // If PID control is enabled, create a MotionMagicVoltage command using the current setpoint
      // and apply it to both motors for smooth positional control. We are setting the setpiont in 
      // rotations but the method of controlling the motors is via voltage rather than percent output
      // hence the name MotionMagicVoltage.
      if (isPIDEnabled) {
          final MotionMagicVoltage setpointWithVoltage = new MotionMagicVoltage(0);
          m_left.setControl(setpointWithVoltage.withPosition(currDesiredSetpoint));
          m_right.setControl(setpointWithVoltage);
          
      }
  
      // Create a MusicTone with 0 frequency (a silent tone) and apply it to both motors.
      // This seems to override any previous motor control commands, ensuring the motors are silent.
      MusicTone silentTone = new MusicTone(0);
      m_left.setControl(silentTone);
      m_right.setControl(silentTone);

      SmartDashboard.putNumber("LEFTTTTTTT", m_left.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("RIGHTTTT", m_right.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Setpoint", currDesiredSetpoint);
      SmartDashboard.putBoolean("At Setpoint", isAtSetpoint());
  }

  /**
   * checks if the elevator is within half a rotation of the setpoint
   * @return if the elevator is within half a rotation of the setpoint
   */
  public boolean isAtSetpoint(){
    System.out.println("is at setpoint" + (getTargetPos() - getPos()));
   return((Math.abs(getTargetPos() - getPos())) < .5);
  }
}
