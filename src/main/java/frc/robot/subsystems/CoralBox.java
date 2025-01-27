package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralBox extends SubsystemBase { 
    private final SparkMax m_left;
    private final SparkMax m_right;
    

    public CoralBox(SparkMax left, SparkMax right){
        m_left = left;
        m_right = right;
    
    }

    /**
     * spins the motors on the coral intake/outake, 
     * range = 0 to 1 coral only needs to go out,
     * 
     * @param speed the speed it spins at
     */
    public void spin(double speed){
        m_left.set(speed);
        m_right.set(speed);

    }

}
