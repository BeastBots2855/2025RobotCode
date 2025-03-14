package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralBox extends SubsystemBase { 
    private final SparkMax m_boxMotor;
    private final TimeOfFlight m_lightSensor;
  
    

    public CoralBox(SparkMax boxMotor){
        m_boxMotor = boxMotor;
        m_lightSensor = new TimeOfFlight(30);
        
    
    }

    /**
     * spins the motors on the coral intake/outake, 
     * range = 0 to 1 coral only needs to go out,
     * 
     * @param speed the speed it spins at
     */
    public void spin(double speed){
        m_boxMotor.set(speed);
        

    }

    public double getDistance(){
        return m_lightSensor.getRange();
    }

    public double getAutoCoralSpeed(){
        return 1.0;
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Dst sensor (smartdashboard)", getDistance());
    }
}
