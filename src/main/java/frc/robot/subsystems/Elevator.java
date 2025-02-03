// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase { 

  private final TalonFX m_left;
  private final TalonFX m_right;
  private final DigitalInput m_limitSwitch;

  /** Creates a new Elevator. */
  public Elevator(TalonFX left, TalonFX right, DigitalInput limitswitch) {
    m_right = right;
    m_left = left;
    m_limitSwitch = limitswitch;
  }

  public void resetEncoders(){
    m_right.setPosition(0);
    m_left.setPosition(0);
  }

  public void setVoltage(double voltage){
    m_right.setVoltage(voltage);
    m_left.setVoltage(voltage);
  }

  public void setPercentOutput(double percentOutput){
    m_left.set(percentOutput);
    m_right.set(percentOutput);
    /**may have to invert later */
  } 

  public void stop(){
    m_left.set(0);
    m_right.set(0);
  }

  public void move(double speed){
    m_left.set(speed);
    m_right.set(speed);
  }
  
  public boolean isLimitSwitchPressed(){
    return m_limitSwitch.get();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
