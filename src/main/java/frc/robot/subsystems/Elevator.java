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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase { 

  private final TalonFXS m_left;
  private final TalonFXS m_right;
  private final DigitalInput m_limitSwitch;
  private boolean currentLimitExceeded;
  

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
    
  }

  public void resetEncoders(){
    m_right.setPosition(0);
    m_left.setPosition(0);
  }

  public void setVoltage(double voltage){
    m_right.setVoltage(voltage);
    m_left.setVoltage(voltage);
  }



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

  public void stop(){
    m_left.set(0);
    m_right.set(0);
  }

  public void move(double speed){
    // if(!currentLimitExceeded){
    m_left.set(speed);
    m_right.set(speed);
    // }
  }
  
  public boolean isLimitSwitchPressed(){
    return m_limitSwitch.get();
  }



  @Override
  public void periodic() {
    if((m_left.getSupplyCurrent().getValueAsDouble() > 20) ||
      m_right.getSupplyCurrent().getValueAsDouble() > 20) {
        currentLimitExceeded = true;
    } else {
      currentLimitExceeded = false;
    }

    // System.out.println(m_left.getSupplyCurrent().getValueAsDouble());
    // This method will be called once per scheduler run
  }
}
