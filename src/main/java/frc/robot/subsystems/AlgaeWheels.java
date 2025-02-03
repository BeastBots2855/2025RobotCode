// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeWheels extends SubsystemBase {
 
  private final SparkMax m_controller;
  
  
  
  /** Creates a new AlgaeWheels. */
  public AlgaeWheels(SparkMax controller){
    m_controller = controller;

  }
  /**
   * Sets speed for motor
   * 
   * @param speed     speed of the motor, range -1 to 1 negative = in, positive = out
   */
  public void spin(double speed){
    
    m_controller.set(speed);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
