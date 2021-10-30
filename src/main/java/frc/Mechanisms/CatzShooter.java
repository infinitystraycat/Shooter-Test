package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Robot;

public class CatzShooter
{
    public WPI_TalonFX shtrMCA;
    public WPI_TalonFX shtrMCB;

    private final int SHTR_MC_ID_A = 10; 
    private final int SHTR_MC_ID_B = 11; 

    final double COUNTS_PER_REVOLUTION      = 2048.0;
    final double SEC_TO_MIN                 = 60.0;
    final double ENCODER_SAMPLE_RATE_MSEC   = 100.0;
    final double ENCODER_SAMPLE_PERIOD_MSEC = (1.0 / ENCODER_SAMPLE_RATE_MSEC);
    final double MSEC_TO_SEC                = 1000.0;
    final double FLYWHEEL_GEAR_REDUCTION    = 3.0;

    final double CONV_QUAD_VELOCITY_TO_RPM = ( ((ENCODER_SAMPLE_PERIOD_MSEC * MSEC_TO_SEC * SEC_TO_MIN) / COUNTS_PER_REVOLUTION)); //converts velocity to RPM

    public static final int SHOOTER_STATE_OFF                 = 0;
    public static final int SHOOTER_STATE_RAMPING             = 1;
    public static final int SHOOTER_WAIT_FOR_STEADY_STATE     = 2;
    public static final int SHOOTER_STATE_READY               = 3;
    public static final int SHOOTER_STATE_START_SHOOTING      = 4;
    public static final int SHOOTER_STATE_WAIT_FOR_SHOOT_DONE = 5;
    
    public final double SHOOTER_RPM_START_OFFSET =  250.0;
    public final double SHOOTER_TARGET_RPM_LO    = 3500;//(4700.0 - SHOOTER_RPM_START_OFFSET); //4400
    public final double SHOOTER_TARGET_RPM_MD    = 4000;//(5000.0 - SHOOTER_RPM_START_OFFSET);
    public final double SHOOTER_TARGET_RPM_HI    = 4500;//(6000.0 - SHOOTER_RPM_START_OFFSET);

    final double SHOOTER_MAX_RPM_OFFSET = 50.0; 
    final double SHOOTER_MIN_RPM_OFFSET = 50.0;

    final double SHOOTER_RAMP_RPM_OFFSET = 1000.0;

    final double SHOOTER_OFF_RPM   =  0.0;
    final double SHOOTER_SHOOT_POWER = 1.0;

    final int NUM_OF_DATA_SAMPLES_TO_AVERAGE = 5;

    final double SHOOTER_THREAD_PERIOD           = 0.040;
    final long   SHOOTER_THREAD_PERIOD_MS        = 40;
    final double SHOOTER_RAMP_TIMEOUT_SEC        = 4.000; 
    final double INDEXER_SHOOT_TIME_SEC          = 3.00;
    final double SHOOTER_AVG_VEL_SAMPLE_TIME_SEC = 0.100;

    public double targetRPM          = 0.0;
    public double targetRPMThreshold = 0.0;
    public double shooterRPM       = 0.0;
    public double minPower           = 0.0;
    public double maxPower           = 0.0;
    public double nomPower           = 0.0;

    public final double kP = 0.0001;

    public static double avgVelocity = 0.0;

    public double lastSamplePeriod = 0.0;
    public double lastEncCounts = 0.0;

    public static int shooterState = SHOOTER_STATE_OFF;

    private int indexerShootStateCountLimit  = 0;
    private int indexerShootStateCount       = 0;

    private boolean shooterIsReady = false;
    public  boolean shooterIsDone  = true;

    private Thread shooterThread;

    public boolean inAutonomous;

    public Timer shootTimer;

    private double minRPM;
    private double maxRPM;

    private double mtrVelocityA = -1.0;
    private double mtrVelocityB = -1.0;

    private double PID_P = 0.3;
    private double PID_I = 0.0;
    private double PID_D = 0.0;
    private double PID_F = (1023.0/21660.0);

    private int PID_IDX_CLOSED_LOOP = 0;
    private int PID_TIMEOUT_MS = 10;

    private int shtrWaitStateCounter = 0;
    private int shtrWaitStateCounterMax = 7;

    private int shooterTraceID = -1;

    public CatzShooter()
    {
        shootTimer = new Timer();

        //initialize motor controllers
        shtrMCA = new WPI_TalonFX(SHTR_MC_ID_A);
        shtrMCB = new WPI_TalonFX(SHTR_MC_ID_B);

        shtrMCA.configFactoryDefault();
        shtrMCB.configFactoryDefault();

        shtrMCA.setNeutralMode(NeutralMode.Coast);
        shtrMCB.setNeutralMode(NeutralMode.Coast);

        shtrMCA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PID_IDX_CLOSED_LOOP, PID_TIMEOUT_MS); //Constants
        shtrMCB.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PID_IDX_CLOSED_LOOP, PID_TIMEOUT_MS);

        shtrMCA.config_kP(0, PID_P);
        shtrMCA.config_kI(0, PID_I);
        shtrMCA.config_kD(0, PID_D);
        shtrMCA.config_kF(0, PID_F);

        shtrMCB.config_kP(0, PID_P);
        shtrMCB.config_kI(0, PID_I);
        shtrMCB.config_kD(0, PID_D);
        shtrMCB.config_kF(0, PID_F);

        //limits how long the shooter runs so it doesn't go too long (limiter)
        indexerShootStateCountLimit = (int)Math.round( (INDEXER_SHOOT_TIME_SEC / SHOOTER_THREAD_PERIOD) + 0.5 ); 
        
        
        shooterOff();
        setShooterVelocity();
        
    }

    public void setShooterVelocity() //will make shooter run and etc
    {

        shooterThread = new Thread(() -> //start of thread
        {

            shootTimer.start();
            double shootTime       = 0.0;
            boolean rumbleSet      = false;

            while(true)
            {
                //System.out.println(shooterState + " : " + shooterTraceID + " : " + shootTime + " : " + shooterRPM + " : " + mtrVelocityA + " : " + mtrVelocityB);
                shootTime = shootTimer.get();
                mtrVelocityA = (Math.abs((double) shtrMCA.getSensorCollection().getIntegratedSensorVelocity() * CONV_QUAD_VELOCITY_TO_RPM));
                mtrVelocityB = (Math.abs((double) shtrMCB.getSensorCollection().getIntegratedSensorVelocity() * CONV_QUAD_VELOCITY_TO_RPM));

                switch (shooterState)
                {
                    case SHOOTER_STATE_OFF: //when there is no targetRPM (basically when no button is pressed) will be shooter most of the time
                       
                        shooterRPM -= 100;
                        if(shooterRPM <= 0)
                        {
                            shooterRPM = SHOOTER_OFF_RPM;
                            //System.out.println("RPM: " + shooterRPM);
                        }
                        setShooterRPM(shooterRPM);
                        //Robot.elevator.stopElevator();

                        if(targetRPM > 0.0)
                        {
                            
                            indexerShootStateCount  = 0;
                            shooterState            = SHOOTER_WAIT_FOR_STEADY_STATE;
                            minRPM                  = targetRPM - SHOOTER_MIN_RPM_OFFSET;
                            maxRPM                  = targetRPM + SHOOTER_MAX_RPM_OFFSET;
                            shtrWaitStateCounter    = 0;
                            rumbleSet               = false;
                            shooterIsDone           = false;
                            
                            shooterRPM = targetRPM;

                            setShooterRPM(targetRPM);

                            shooterTraceID = 1;
                        }

                    break;

                    case SHOOTER_WAIT_FOR_STEADY_STATE: // checks if the rpm is within the threshold for n numbers of times consecutively
                        shooterTraceID = 20;

                        if(mtrVelocityA > minRPM && mtrVelocityA < maxRPM)
                        {
                            shooterTraceID = 21;
                            shtrWaitStateCounter ++;
                            if(shtrWaitStateCounter >= shtrWaitStateCounterMax)
                            {
                                shooterTraceID = 22;
                                shooterState = SHOOTER_STATE_READY;
                            }
                        }else{
                            shooterTraceID = 23;
                            shtrWaitStateCounter = 0;
                        }
                    break;

                    case SHOOTER_STATE_READY:// makes the controller vibrate so that aux driver knows to shoot
                        shooterTraceID = 30;
                        shooterIsReady = true;

                        if(inAutonomous == true)
                        { 
                            
                            shooterTraceID = 31;
                            shoot();
                        }
                        else 
                        {
                            shooterTraceID = 32;
                            if(rumbleSet == false)
                            {
                                shooterTraceID = 33;
                                Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 0.5);
                                rumbleSet = true;
                            } 
                        }
                    break;

                    case SHOOTER_STATE_WAIT_FOR_SHOOT_DONE: //will count for a certain amount of time until it switches the shooter off and sets state to OFF
                        shooterTraceID = 40;
                        indexerShootStateCount++;
                        if(indexerShootStateCount > indexerShootStateCountLimit)
                        {
                            shooterTraceID = 41;
                            shooterOff();
                        }
                
                    break;

                    default:  //default code when there is nothing going on 
                        shooterOff();
                    break;
                }        
                
                Timer.delay(SHOOTER_THREAD_PERIOD);

            }
        }); //end of thread
        shooterThread.start();
    }

    public void setShooterRPM(double rpm)
    {
        double velocityB = rpm * (2048.0) * (1.0/60.0) * (1.0/10.0);
        double velocityA = -velocityB; //changed from negative to pos
        
        shtrMCA.set(TalonFXControlMode.Velocity, velocityA);
        shtrMCB.set(TalonFXControlMode.Velocity, velocityB);
    }

    public void setTargetRPM(double velocity) // Sets the RPM (determined by the button pressed on controller)
    {
        targetRPM = velocity;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    }

    public void shoot() // when a button is pressed on controller, it will cause the indexer to move and launch balls. sets state to ready
    {
        if(shooterState == SHOOTER_STATE_READY)
        {
            indexerShootStateCount = 0;
            Robot.elevator.runElevator();
            shooterState = SHOOTER_STATE_WAIT_FOR_SHOOT_DONE;
        }
    }

    public void shooterOff() // turns shooter off , sets the shooter state to off
    {
        shooterState   = SHOOTER_STATE_OFF;
        targetRPM      = 0.0;
        shooterIsReady = false;
	    shooterIsDone  = true;
        
        Robot.elevator.stopElevator();
        //shooterRPM   = SHOOTER_OFF_RPM;
        //setShooterRPM(shooterRPM);
        //Robot.indexer.setShooterIsRunning(false);
        Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 0);
    
    }

    public double getRPM()
    {
        return mtrVelocityA;
    }
}