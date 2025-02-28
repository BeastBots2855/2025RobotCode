// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
  /** Creates a new AlgaeArm. */
  private final SparkMax m_AlgaeArmMotor;
  public AlgaeArm(SparkMax motor) {
    m_AlgaeArmMotor = motor;
  }

  public void move(Double speed){
    speed *= .25;
    m_AlgaeArmMotor.set(speed);
  }

  public void stop(){
    m_AlgaeArmMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
