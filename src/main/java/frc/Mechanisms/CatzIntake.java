package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DataLogger.CatzLog;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

public class CatzIntake
{
    public WPI_TalonSRX intakeRollerMC;
    public CANSparkMax intakeDeployMC;

    //public CANEncoder intakeDeployEncoderLT;
    public CANEncoder intakeDeployEncoderRT;

    private final int INTAKE_ROLLER_MC_CAN_ID = 31;
    private final int INTAKE_DEPLOY_MC_CAN_ID = 38;//30; 

    private int deployPowerCountLimit = 0;
    private int stowPowerCountLimit   = 0;
    private int timeCounter = 0;

    private final double MTR_POWER_ROLLER =  1.0;

    private final double INTAKE_MOTOR_POWER_START_DEPLOY =  0.15;
    private final double INTAKE_MOTOR_POWER_END_DEPLOY   =  0.0;
    private final double INTAKE_MOTOR_POWER_START_STOW   = -0.40; //originally -0.40
    private final double INTAKE_MOTOR_POWER_END_STOW     = -0.05;

    private final double INTAKE_ENCODER_CONV_FACTOR = 7.0 * 7.0;

    private Thread intakeThread;

    private final int INTAKE_MODE_NULL                = 0;
    private final int INTAKE_MODE_DEPLOY_START        = 1;
    private final int INTAKE_MODE_DEPLOY_REDUCE_POWER = 2;
    private final int INTAKE_MODE_STOW_START          = 3;
    private final int INTAKE_MODE_STOW_REDUCE_POWER   = 4;
    private final int INTAKE_MODE_WAIT_FOR_HARD_STOP  = 5;
    private final int INTAKE_MODE_STOW_HOLD           = 6;

    private int intakeMode = INTAKE_MODE_NULL;

    boolean intakeDeployed = false;

    final double INTAKE_THREAD_WAITING_TIME       = 0.020;
    final double DEPLOY_REDUCE_POWER_TIME_OUT_SEC = 0.400;
    final double STOW_REDUCE_POWER_TIME_OUT_SEC   = 0.250; //originally 0.450

    final double intakeStowMotorGearRatio = 1.0/49.0;

    private double INTAKE_MIN_DEGREE = 1.524202; //give some leeway degrees -david
    private double INTAKE_MIN_SPEED_CHANGE_DEGREE = 10.00;
    private double INTAKE_MAX_DEGREE = 80.466894; //give some leeway degrees -david
    private double INTAKE_MAX_SPEED_CHANGE_DEGREE = 70.00;

    private double currentIntakeStowPosition;
    private double lastIntakeStowPosition;
    private double targetIntakeStowPosition;
    private double currentIntakeStowPower;
    private int intakeCheckHardstopCount = 0;

    private final int INTAKE_MAX_HARD_STOP_COUNT          = 7;
    private final double INTAKE_STOW_HOLD_THRESHOLD       = 15.0;
    private final double INTAKE_CHECK_HARD_STOP_THRESHOLD = 1.0;

    private double intakeStowHoldkP = 0.01;

    private boolean dataCollectionState = true; //true for on, flase for off. Turns on or off datacollection for this class
    private CatzLog data;

    private Timer intakeTimer;

    private double error = 0.0;
    private double power = 0.0;
    private double currentTime = 0.0;
    private double intakeMtrPwr = 0.0;
    private double currentPosition = 0.0;

    public CatzIntake()
    {
        intakeRollerMC = new WPI_TalonSRX(INTAKE_ROLLER_MC_CAN_ID);
        intakeDeployMC = new CANSparkMax(INTAKE_DEPLOY_MC_CAN_ID, MotorType.kBrushless);
        
        intakeDeployEncoderRT = new CANEncoder(intakeDeployMC);

        //Reset configuration
        intakeRollerMC.configFactoryDefault();
        intakeDeployMC.restoreFactoryDefaults();


        //Set roller MC to coast intakeMode
        intakeRollerMC.setNeutralMode(NeutralMode.Coast);

        //Set deploy MC to brake intakeMode
        intakeDeployMC.setIdleMode(IdleMode.kBrake);

        deployPowerCountLimit = (int) Math.round((DEPLOY_REDUCE_POWER_TIME_OUT_SEC / INTAKE_THREAD_WAITING_TIME) + 0.5);
        stowPowerCountLimit   = (int) Math.round((STOW_REDUCE_POWER_TIME_OUT_SEC   / INTAKE_THREAD_WAITING_TIME) + 0.5);
        SmartDashboard.putNumber("stow count limit", stowPowerCountLimit);
        SmartDashboard.putNumber("deploy count limit", deployPowerCountLimit);

        intakeTimer = new Timer();
    }

    // ---------------------------------------------ROLLER---------------------------------------------
    public void intakeRollerIn()
    {
        intakeRollerMC.set(ControlMode.PercentOutput, -MTR_POWER_ROLLER);
    }
    public void intakeRollerOut()
    {
        intakeRollerMC.set(ControlMode.PercentOutput, MTR_POWER_ROLLER);
    }
    public void intakeRollerOff()
    {
        intakeRollerMC.set(ControlMode.PercentOutput, 0.0);
    }

    // ---------------------------------------------DEPLOY/STOW---------------------------------------------
    public void intakeControl()
    {

        intakeThread = new Thread(() ->
        { 
            intakeTimer.reset();
            intakeTimer.start();
            while(true)
            {
                currentPosition = getIntakeDeployPositionDegrees();
                currentTime = intakeTimer.get();//add
                System.out.println("Timer: " + currentTime + " Mode: " + intakeMode + " pos deg:" + getIntakeDeployPositionDegrees() + " power: " + power);//add
                switch(intakeMode)
                {
                    case INTAKE_MODE_DEPLOY_START:
                        //set position as 0 later (-david)
                        setIntakeMotorPower(INTAKE_MOTOR_POWER_START_DEPLOY);
                        intakeMode = INTAKE_MODE_STOW_REDUCE_POWER;
                    
                    break;

                    case INTAKE_MODE_DEPLOY_REDUCE_POWER:
                        if(getIntakeDeployPositionDegrees() <= INTAKE_MIN_SPEED_CHANGE_DEGREE)
                        {
                            setIntakeMotorPower(INTAKE_MOTOR_POWER_END_DEPLOY);
                        }
                        else if(getIntakeDeployPositionDegrees() <= INTAKE_MIN_DEGREE)
                        {
                            intakeMode = INTAKE_MODE_NULL;
                        }
                    break;

                    case INTAKE_MODE_STOW_START:
                        setIntakeMotorPower(INTAKE_MOTOR_POWER_START_STOW);
                        intakeMode = INTAKE_MODE_STOW_REDUCE_POWER;
                    break;

                    case INTAKE_MODE_STOW_REDUCE_POWER:
                        if(getIntakeDeployPositionDegrees() >= INTAKE_MAX_SPEED_CHANGE_DEGREE)
                        {
                            setIntakeMotorPower(INTAKE_MODE_STOW_REDUCE_POWER);
                            intakeCheckHardstopCount = 0;
                            intakeMode = INTAKE_MODE_STOW_START;
                            intakeDeployed = false;
                        }
                    break;

                    case INTAKE_MODE_WAIT_FOR_HARD_STOP:
                        if(currentIntakeStowPosition >= INTAKE_MAX_DEGREE)
                        {
                            targetIntakeStowPosition = currentIntakeStowPosition;
                            currentIntakeStowPower = INTAKE_MOTOR_POWER_END_STOW;
                            intakeMode = INTAKE_MODE_STOW_HOLD;
                        }
                    break;

                    case INTAKE_MODE_STOW_HOLD:
                        currentIntakeStowPosition = getIntakeDeployPositionDegrees();
                        error = currentIntakeStowPosition - targetIntakeStowPosition;

                        if(Math.abs(error) >= INTAKE_STOW_HOLD_THRESHOLD)
                        {
                            System.out.println("IN power change");
                            power = clamp(currentIntakeStowPower + (error * intakeStowHoldkP), -1.0, INTAKE_MOTOR_POWER_END_STOW);
                            currentIntakeStowPower = power;
                            setIntakeMotorPower(power);
                        }
                        else
                        {
                            setIntakeMotorPower(0.0);
                        }

                    break;

                    default:
                        setIntakeMotorPower(0.0);
                    break;
                }
                
                if(dataCollectionState)
                {
                    data = new CatzLog(currentTime, (double)intakeMode, intakeMtrPwr, targetIntakeStowPosition, currentPosition, error, 
                                        intakeRollerMC.getBusVoltage(), intakeRollerMC.getStatorCurrent() ,intakeDeployMC.getBusVoltage(), intakeDeployMC.getOutputCurrent(), getIntakeDeployPositionDegrees(), -999.0, -999.0, -999.0, -999.0, -999.0);
                    Robot.dataCollection.logData.add(data);
                }
                Timer.delay(INTAKE_THREAD_WAITING_TIME); //put at end
                
            }   
        });
        intakeThread.start();
    }
    public void deployIntake()
    {
        timeCounter = 0;
        intakeMode = INTAKE_MODE_DEPLOY_START;
    }
    public void stowIntake()
    {
        timeCounter = 0;
        intakeMode = INTAKE_MODE_STOW_START;
    }

    public double getIntakeDeployPositionDegrees(){
        return Math.abs(intakeDeployEncoderRT.getPosition() * intakeStowMotorGearRatio * 360.0);
    }

    public double clamp(double value, double min, double max){
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    public void setIntakeMotorPower(double power)
    {
        intakeDeployMC.set(-power);
        intakeMtrPwr = power; 
    }

    public double convertEnc()
    {
       double c = (intakeDeployEncoderRT.getPosition())*INTAKE_ENCODER_CONV_FACTOR;
       return c;
    }
}