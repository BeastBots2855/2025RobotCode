// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPIDSetpoints;


public class Elevator extends SubsystemBase { 

  private final TalonFXS m_left;
  private final TalonFXS m_right;
  private final DigitalInput m_limitSwitch;
  private boolean currentLimitExceeded;
  private PIDController m_PIDController;
  private double currDesiredSetpoint;
  private boolean isPIDEnabled;
  //private SimpleMotorFeedforward m_Feedforward;

  //setpoints to keep elevator stationary, and store the calculation, in that order
  //private double breakModeSetpoint;
  //private double feedForwardSetpoint; 

  /** Creates a new Elevator. */
  public Elevator(TalonFXS left, TalonFXS right, DigitalInput limitswitch) {
    m_right = right;
    m_left = left;
    m_limitSwitch = limitswitch;
    m_PIDController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

    m_left.setNeutralMode(NeutralModeValue.Brake);
    m_right.setNeutralMode(NeutralModeValue.Brake);
    


    

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


    MusicTone obnoxiousAsHellTone = new MusicTone(20000);
    m_left.setControl(obnoxiousAsHellTone);
    m_right.setControl(obnoxiousAsHellTone);
    
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    tab.add(m_left);
    tab.add(m_right);
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


  public void setSetpoint(double setpoint){
    //re-enable
    currDesiredSetpoint = setpoint;
    m_PIDController.setSetpoint(setpoint);
  }








 

  public void stop(){
    m_left.set(0);
    m_right.set(0);
  }

    public double getOutput(){
      return m_left.getDutyCycle().getValueAsDouble();
    }
  public void PIDOff(){
    isPIDEnabled = false;
  }

  public void PIDOn(){
    isPIDEnabled = true;
  }

  public void move(double speed){
    isPIDEnabled = false;



    if(m_limitSwitch.get() && speed < 0){
      speed = 0;
      m_left.set(0);
     m_right.set(0);
     System.out.println("at bottom");
    }else{
      m_left.set(speed *= .25);
      m_right.set(speed *= .25);
      System.out.println("running");
    }
  }
  
  public boolean isLimitSwitchPressed(){
    return m_limitSwitch.get();
  }
  public double getPos(){
    return m_left.getPosition().getValueAsDouble();
  }
  public double getTargetPos(){
    return currDesiredSetpoint;
  }
 


  @Override
  public void periodic() {
    currentLimitExceeded = ((m_left.getSupplyCurrent().getValueAsDouble() > 20) ||
      m_right.getSupplyCurrent().getValueAsDouble() > 20);


    final PositionDutyCycle m_request = new PositionDutyCycle(0).withSlot(0);

    if (isPIDEnabled){

     m_left.set(m_PIDController.calculate(getPos()) + ElevatorConstants.feedForward);
     m_right.set(m_PIDController.calculate(getPos()) + ElevatorConstants.feedForward);
    // move(m_PIDController.calculate(getPos() + ElevatorConstants.feedForward));
      //m_left.setControl(m_request.withPosition(currDesiredSetpoint));
     // m_right.setControl(m_request.withPosition(currDesiredSetpoint));
    }
  }
}
