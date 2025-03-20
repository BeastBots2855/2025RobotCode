// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final SparkMax m_right;
  private final SparkMax m_left; 
  private final AbsoluteEncoder m_Encoder;
  /** Creates a new Climb. */
  public Climber(SparkMax right, SparkMax left) {
    m_right = right;
    m_left = left;
    m_Encoder = m_left.getAbsoluteEncoder();
  }

  public void spin(double speed){
    m_right.set(-speed);
    m_left.set(speed);
  }

  public double getEncoder(){
    return m_Encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
