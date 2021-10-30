package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class CatzElevator
{
    public WPI_TalonSRX elvtrMCA;
    public WPI_VictorSPX elvtrMCB;

    private final int ELVTR_MC_ID_A = 20; 
    private final int ELVTR_MC_ID_B = 21; 

    private final double ELE_POWER = 0.5;
    private final double ELE_FACTOR = 1.3;

    public CatzElevator()
    {
        elvtrMCA = new WPI_TalonSRX(ELVTR_MC_ID_A);
        elvtrMCB = new WPI_VictorSPX(ELVTR_MC_ID_B);
    }

    public void runElevator(){
        elvtrMCA.set(-ELE_POWER);
        elvtrMCB.set(ELE_POWER * ELE_FACTOR);
    }

    public void stopElevator(){
        elvtrMCA.set(0);
        elvtrMCB.set(0);
    }
}