// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;


public class Elevator extends SubsystemBase { 

  private final TalonFXS m_left;
  private final TalonFXS m_right;
  private final DigitalInput m_limitSwitch;
  

  /** Creates a new Elevator. */
  public Elevator(TalonFXS left, TalonFXS right, DigitalInput limitswitch) {
    m_right = right;
    m_left = left;
    m_limitSwitch = limitswitch;

    //sets up configurator/configuration for both elevator motors
    TalonFXSConfigurator leftTalonFXSConfigurator = m_left.getConfigurator();
    TalonFXSConfiguration leftTalonConfiguration = new TalonFXSConfiguration();
    TalonFXSConfigurator rightTalonFXSConfigurator = m_right.getConfigurator();
    TalonFXSConfiguration rightTalonConfiguration = new TalonFXSConfiguration();

    //inverts and applies
    leftTalonConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftTalonFXSConfigurator.apply(leftTalonConfiguration);
    rightTalonConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightTalonFXSConfigurator.apply(rightTalonConfiguration);
    
    //Experimental - should hopefully display subsystem in shuffleboard for easier tuning
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    tab.add(m_right);
    tab.add(m_left);
    tab.add(m_limitSwitch);
  }

  public void resetEncoders(){
    m_right.setPosition(0);
    m_left.setPosition(0);
  }

  public void setVoltage(double voltage){
    m_right.setVoltage(voltage);
    m_left.setVoltage(voltage);
  }
// ^ this does the same thing as 'move' but with different units, do we need all these control methods?


  /**
   * sets the percent output for both motors with positive values drive elevator up
   * 
   * @param percentOutput the percent value to set the motor output to
   * returns void
   */
  public void setPercentOutput(double percentOutput){
    m_left.set(percentOutput);
    m_right.set(percentOutput);
  } 
  // ^ this is exactly the same as 'move' but without the current safeguard, does it need to be here?

  public void stop(){
    m_left.set(0);
    m_right.set(0);
  }

  public void move(double speed){
    if(!isCurrentExceeded()){
      m_left.set(speed);
      m_right.set(speed);
    }
  }
  
  public boolean isLimitSwitchPressed(){
    return m_limitSwitch.get();
  }

  /**
   * Whether or not either motor is currently exceeding the maximum allowed supply current
   * specified in <code>Constants.ElevatorConstants.kMaxCurrent</code>
   * @return true if either motor exceeds kMaxCurrent
   */
  public boolean isCurrentExceeded(){
    return m_left.getSupplyCurrent().getValueAsDouble() > ElevatorConstants.kMaxCurrent ||
            m_right.getSupplyCurrent().getValueAsDouble() > ElevatorConstants.kMaxCurrent;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
