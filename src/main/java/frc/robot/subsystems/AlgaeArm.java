// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeArmConstants;



public class AlgaeArm extends SubsystemBase {
  /** Creates a new AlgaeArm. */
  private final SparkMax m_AlgaeArmMotor;
  private final RelativeEncoder m_RelativeEncoder;
  private double targetSetpoint;
  private SparkMaxConfig m_AlgaeMotorConfig;
  private PIDController m_PIDController;
  private boolean PIDEnabled;

  

  public AlgaeArm(SparkMax motor) {
    m_AlgaeArmMotor = motor;
    m_RelativeEncoder = m_AlgaeArmMotor.getEncoder();
    m_AlgaeMotorConfig = new SparkMaxConfig();
    m_AlgaeMotorConfig.inverted(true);
    m_AlgaeArmMotor.configure(m_AlgaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_PIDController = new PIDController(AlgaeArmConstants.kP, AlgaeArmConstants.kI, AlgaeArmConstants.kD);
  }

  public void move(Double speed){
    PIDOff();
    speed *= .25;
    m_AlgaeArmMotor.set(speed);
  }

  public void stop(){
    m_AlgaeArmMotor.set(0);
  }

  public double getCurrent(){
    return m_AlgaeArmMotor.getOutputCurrent();
  }

  public void setSetpoint(double setpoint){
    targetSetpoint = setpoint;
  }

  public double getSetpoint(){
    return targetSetpoint;
  }

  public double getPos(){
    return m_RelativeEncoder.getPosition();
  }

  public void resetPosition(){
    m_RelativeEncoder.setPosition(0.0);
  }

  public void PIDOn(){
    PIDEnabled = true;
  }

  public void PIDOff(){
    PIDEnabled = false;
  }
  

  @Override
  public void periodic() {
    if(PIDEnabled){
     m_AlgaeArmMotor.set(m_PIDController.calculate(getPos(), targetSetpoint));
    } 

    SmartDashboard.putNumber("Algae Encoder Pos", m_RelativeEncoder.getPosition());
    SmartDashboard.putNumber("Algae Current", m_AlgaeArmMotor.getOutputCurrent());
  }
}
