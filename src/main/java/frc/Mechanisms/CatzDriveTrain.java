package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Robot;

public class CatzDriveTrain
{
    public WPI_TalonFX drvTrainMtrCtrlLTFrnt;
    public WPI_TalonFX drvTrainMtrCtrlLTBack;
    public WPI_TalonFX drvTrainMtrCtrlRTFrnt;
    public WPI_TalonFX drvTrainMtrCtrlRTBack;

    public final int DRVTRAIN_LT_FRNT_MC_CAN_ID = 1;//1;
    public final int DRVTRAIN_LT_BACK_MC_CAN_ID = 2;//2;

    public final int DRVTRAIN_RT_FRNT_MC_CAN_ID = 3;//3;
    public final int DRVTRAIN_RT_BACK_MC_CAN_ID = 4;//4;

    public final int DRV_TRN_LT_FRNT_MC_PDP_PORT = 0;
    public final int DRV_TRN_LT_BACK_MC_PDP_PORT = 1;
    public final int DRV_TRN_RT_FRNT_MC_PDP_PORT = 15;
    public final int DRV_TRN_RT_BACK_MC_PDP_PORT = 14;

    private DifferentialDrive drvTrainDifferentialDrive;

    public SpeedControllerGroup drvTrainLT;///TBD change back to private
    public SpeedControllerGroup drvTrainRT;

    private DoubleSolenoid gearShifter;

    private final int DRVTRAIN_LGEAR_SOLENOID_PORT_A_PCM = 3;
    private final int DRVTRAIN_HGEAR_SOLENOID_PORT_B_PCM = 4;

    
    //private final double GEAR_RATIO    = 11/44;  TBD - OK TO LEAVE AS COMMENT IF DATA IS CORRECT
    private final double LOW_GEAR_RATIO  = 857.0/50.0;
    private final double HIGH_GEAR_RATIO = 25.0/3.0;

    private final double TALONFX_INTEGRATED_ENC_CNTS_PER_REV      = 2048.0;
    private final double DRVTRAIN_WHEEL_RADIUS                    = 3.0;

    public double encCountsToInches = 0;
    public final double DRVTRAIN_ENC_COUNTS_TO_INCHES_HI = (1/HIGH_GEAR_RATIO) * (2 * Math.PI * DRVTRAIN_WHEEL_RADIUS) * (1/TALONFX_INTEGRATED_ENC_CNTS_PER_REV) ; //do calculations, method to determine gear and apply conversion factor
    public final double DRVTRAIN_ENC_COUNTS_TO_INCHES_LO = (1/ LOW_GEAR_RATIO) * (2 * Math.PI * DRVTRAIN_WHEEL_RADIUS) * (1/TALONFX_INTEGRATED_ENC_CNTS_PER_REV) ;

    public int currentDrvTrainGear;    

    public final int DRV_TRAIN_GEAR_LO = 0;
    public final int DRV_TRAIN_GEAR_HI = 1;
   

    private final int PRESSURE_SENSOR_ANALOG_PORT       = 3; 

    private final double PRESSURE_SENSOR_VOLTAGE_OFFSET = 0.5;

    private final double PRESSURE_SENSOR_VOLATGE_RANGE  = 4.5;    //4.5-0.5
    private final double MAX_PRESSURE                   = 200.0;

    private AnalogInput pressureSensor;

    private SupplyCurrentLimitConfiguration drvTrainCurrentLimit;
    private StatorCurrentLimitConfiguration drvTrainStatorCurrentLimit;

    private boolean enableCurrentLimit              = true; 
    private final int CURRENT_LIMIT_AMPS            = 60;
    private final int CURRENT_LIMIT_TRIGGER_AMPS    = 80;
    private final int CURRENT_LIMIT_TIMEOUT_SECONDS = 5;

    private final int DRVTRAIN_VELOCITY_PID_IDX = 0;
    private final int PID_TIMEOUT_MS      = 10;


    public final double RT_PID_P = 0.1;  
    public final double RT_PID_I = 0.0; 
    public final double RT_PID_D = 0.0;    
    public final double RT_PID_F = 1023.0/20666.0; 

    public final double LT_PID_P = 0.1;//orig 0.1;  
    public final double LT_PID_I = 0.0; 
    public final double LT_PID_D = 0.0;    
    public final double LT_PID_F = 1023.0/20666.0;

    public final double DRV_TRAIN_RT_VELOCITY_OFFSET = 0.0;

    public CatzDriveTrain() 
    {
        drvTrainMtrCtrlLTFrnt = new WPI_TalonFX(DRVTRAIN_LT_FRNT_MC_CAN_ID);
        drvTrainMtrCtrlLTBack = new WPI_TalonFX(DRVTRAIN_LT_BACK_MC_CAN_ID);

        drvTrainMtrCtrlRTFrnt = new WPI_TalonFX(DRVTRAIN_RT_FRNT_MC_CAN_ID);
        drvTrainMtrCtrlRTBack = new WPI_TalonFX(DRVTRAIN_RT_BACK_MC_CAN_ID);

        //Reset configuration for drivetrain MC's
        drvTrainMtrCtrlLTFrnt.configFactoryDefault();
        drvTrainMtrCtrlLTBack.configFactoryDefault();
        drvTrainMtrCtrlRTFrnt.configFactoryDefault();
        drvTrainMtrCtrlRTBack.configFactoryDefault();

        //Set current limit
        drvTrainCurrentLimit = new SupplyCurrentLimitConfiguration(enableCurrentLimit, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);
        

        drvTrainMtrCtrlLTFrnt.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlLTBack.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlRTFrnt.configSupplyCurrentLimit(drvTrainCurrentLimit);
        drvTrainMtrCtrlRTBack.configSupplyCurrentLimit(drvTrainCurrentLimit);

        //Set back Motor Controllers to follow front Motor Controllers
        drvTrainMtrCtrlLTBack.follow(drvTrainMtrCtrlLTFrnt);
        drvTrainMtrCtrlRTBack.follow(drvTrainMtrCtrlRTFrnt);

        //Set MC's in brake mode
        drvTrainMtrCtrlLTFrnt.setNeutralMode(NeutralMode.Brake); 
        drvTrainMtrCtrlLTBack.setNeutralMode(NeutralMode.Brake);
        drvTrainMtrCtrlRTFrnt.setNeutralMode(NeutralMode.Brake);
        drvTrainMtrCtrlRTBack.setNeutralMode(NeutralMode.Brake);

        //drvTrainStatorCurrentLimit = new StatorCurrentLimitConfiguration(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);//consider configuring value ConfigStatorCurrentLimit()
        //drvTrainMtrCtrlLTFrnt.configStatorCurrentLimit(drvTrainStatorCurrentLimit);

        drvTrainLT = new SpeedControllerGroup(drvTrainMtrCtrlLTFrnt, drvTrainMtrCtrlLTBack);
        drvTrainRT = new SpeedControllerGroup(drvTrainMtrCtrlRTFrnt, drvTrainMtrCtrlRTBack);

        gearShifter = new DoubleSolenoid(DRVTRAIN_LGEAR_SOLENOID_PORT_A_PCM, DRVTRAIN_HGEAR_SOLENOID_PORT_B_PCM);

        pressureSensor = new AnalogInput(PRESSURE_SENSOR_ANALOG_PORT);

        setDriveTrainPIDConfiguration();

        shiftToHighGear();
    }

    public void setDriveTrainPIDConfiguration() 
    {
         //Configure feedback device for PID loop
         drvTrainMtrCtrlLTFrnt.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, DRVTRAIN_VELOCITY_PID_IDX, PID_TIMEOUT_MS); //Constants
         drvTrainMtrCtrlRTFrnt.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, DRVTRAIN_VELOCITY_PID_IDX, PID_TIMEOUT_MS);

         //Configure PID Gain Constants
         drvTrainMtrCtrlLTFrnt.config_kP(DRVTRAIN_VELOCITY_PID_IDX, LT_PID_P);
         drvTrainMtrCtrlLTFrnt.config_kI(DRVTRAIN_VELOCITY_PID_IDX, LT_PID_I);
         drvTrainMtrCtrlLTFrnt.config_kD(DRVTRAIN_VELOCITY_PID_IDX, LT_PID_D);
         drvTrainMtrCtrlLTFrnt.config_kF(DRVTRAIN_VELOCITY_PID_IDX, LT_PID_F);

         drvTrainMtrCtrlRTFrnt.config_kP(DRVTRAIN_VELOCITY_PID_IDX, RT_PID_P);
         drvTrainMtrCtrlRTFrnt.config_kI(DRVTRAIN_VELOCITY_PID_IDX, RT_PID_I);
         drvTrainMtrCtrlRTFrnt.config_kD(DRVTRAIN_VELOCITY_PID_IDX, RT_PID_D);
         drvTrainMtrCtrlRTFrnt.config_kF(DRVTRAIN_VELOCITY_PID_IDX, RT_PID_F);
    }

    public void arcadeDrive(double power, double rotation)
    {
       drvTrainDifferentialDrive.arcadeDrive(-power, rotation);
    }

    public void shiftToHighGear()
    {
        gearShifter.set(Value.kForward);
        currentDrvTrainGear = DRV_TRAIN_GEAR_HI; //curr
        encCountsToInches = DRVTRAIN_ENC_COUNTS_TO_INCHES_HI;
    }

    public void shiftToLowGear()
    {
        gearShifter.set(Value.kReverse);
        currentDrvTrainGear = DRV_TRAIN_GEAR_LO;
        encCountsToInches = DRVTRAIN_ENC_COUNTS_TO_INCHES_LO;
    }

    public double getMotorTemperature(int id)
    {
        double temp = 0.0;
        if(id == DRVTRAIN_LT_FRNT_MC_CAN_ID)
        {
            temp = drvTrainMtrCtrlLTFrnt.getTemperature();
        } 
        else if (id == DRVTRAIN_LT_BACK_MC_CAN_ID)
        {   
            temp = drvTrainMtrCtrlLTBack.getTemperature();
        }
        else if (id == DRVTRAIN_RT_FRNT_MC_CAN_ID)
        {
            temp = drvTrainMtrCtrlRTFrnt.getTemperature();
        }
        else if (id == DRVTRAIN_RT_BACK_MC_CAN_ID)
        {
            temp = drvTrainMtrCtrlRTBack.getTemperature();
        }
        return temp;
    }

    public int getDrvTrainEncoderCnt(String side) 
    {
        int counts = 0;
        side.toUpperCase();
        if(side.equals("LT"))
        {
            counts = drvTrainMtrCtrlLTFrnt.getSelectedSensorPosition(0);
        }
        else // if(side.equals("RT"))
        {
            counts = drvTrainMtrCtrlRTFrnt.getSelectedSensorPosition(0);
        }
        return counts;
    }

    public double getIntegratedEncVelocity(String side)
    {
        double velocity = 0.0;
        side.toUpperCase();
        if(side.equals("LT"))
        {
            velocity = drvTrainMtrCtrlLTFrnt.getSensorCollection().getIntegratedSensorVelocity(); //returning counts per 100ms
        }
        else if(side.equals("RT"))
        {
            velocity = drvTrainMtrCtrlRTFrnt.getSensorCollection().getIntegratedSensorVelocity();
        }
        return velocity;
    }

    public void setTargetPosition(double targetPosition)
    {
        drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Position, targetPosition);
        drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Position, targetPosition);
    }

    public void setTargetVelocity(double targetVelocity)
    {
        double tarVelRT = -(targetVelocity - DRV_TRAIN_RT_VELOCITY_OFFSET);
        
        drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, targetVelocity);
        drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, tarVelRT);
        
    }

    public void setTurnInPlaceVelocity(double velocity) 
    {
        drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.Velocity, (velocity));
        drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.Velocity, (velocity));  
    }

    public double getVelocityError(String side, double targetVelocity)
    {
        double velocityError = 0.0;
        if(side.equals("LT"))
        {
            velocityError = targetVelocity - getIntegratedEncVelocity("LT");
        }
        else if(side.equals("RT"))
        {
            velocityError = targetVelocity - getIntegratedEncVelocity("RT");
        }
        return velocityError;
    }

    public void setIntegratedEncPosition(int position)
    {
        drvTrainMtrCtrlLTFrnt.setSelectedSensorPosition(position);
    }

    public double convertLinearVelocityToAngularVelcoity(double linearVelocity)
    {
        return linearVelocity*12.0/DRVTRAIN_WHEEL_RADIUS/(2*Math.PI)*TALONFX_INTEGRATED_ENC_CNTS_PER_REV/1000.0*100.0;
    }

    public double getPSI(double voltage)
    {
      voltage = pressureSensor.getVoltage() - PRESSURE_SENSOR_VOLTAGE_OFFSET;
      return (MAX_PRESSURE/PRESSURE_SENSOR_VOLATGE_RANGE) *voltage;  
    }   

    public void instantiateDifferentialDrive()
    {
        drvTrainDifferentialDrive = new DifferentialDrive(drvTrainLT, drvTrainRT);
    }
    
    public void setTargetPower(double targetPower) {
        drvTrainMtrCtrlLTFrnt.set(TalonFXControlMode.PercentOutput, (targetPower));
        drvTrainMtrCtrlRTFrnt.set(TalonFXControlMode.PercentOutput, (targetPower));  
	}
}