/**
 * This file handles main for controlling rover functions.
 * 
 * Version: 23012022
 * 
 * Author: Jithin George
 */

/* Includes. */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>
#include <mutex>
/* Brett's PID library. */
#include "PID_v1.h"
#include "motorDriver.h"
#include "sensorDriver.h"
#include "AWS.h"

/* Public Defines. */
/* Debug prints. */
//#define SENSOR_VAL_DEBUG
//#define MOTOR_SPEED_DEBUG
//#define TASK_SCHEDULE_DEBUG
//#define LINEAR_SPEED_REDUCTION
//#define AWS_SUB_DEBUG
//#define SENSOR_BORDER_TRIGGER_DEBUG
//#define ROVER_MANUAL_CONTROL
#define ROVER_BATTERY_POWER

/* Pin of the board to V2. */
#define LED_BOARD  (0x02U)

/* Delays. */
#define DELAY_30    (pdMS_TO_TICKS(0x1EU))
#define DELAY_100   (pdMS_TO_TICKS(0x64U))
#define DELAY_200   (pdMS_TO_TICKS(0xC8U))
#define DELAY_500   (pdMS_TO_TICKS(0x1F4U))
#define DELAY_1000  (pdMS_TO_TICKS(0x3E8U))
#define DELAY_3000  (pdMS_TO_TICKS(3000U))
#define DELAY_DEBUG (5000U)

/* Motor speeds. */
#define MOTOR_LOCK       (0x00U)
#define MOTOR_LOW        (0x0AU)
#ifdef ROVER_BATTERY_POWER
#define MOTOR_HIGH_TURN  (75U)
#define MOTOR_HIGH_FWD   (125U)
#define PID_MOTOR_MIN_TURN (-95)
#define PID_MOTOR_MAX_TURN (95)
#define PID_MOTOR_MIN_FWD  (-165)
#define PID_MOTOR_MAX_FWD  (165)
#else
#define MOTOR_HIGH_TURN    (100U)
#define MOTOR_HIGH_FWD     (150U)
#define PID_MOTOR_MIN_TURN (-165)
#define PID_MOTOR_MAX_TURN (165)
#define PID_MOTOR_MIN_FWD  (-200)
#define PID_MOTOR_MAX_FWD  (200)
#endif
#define MOTOR_MAX        (0xFFU)
#define MOTOR_STEP_SIZE  (0x0AU)
#define MOTOR_STEP_DELAY (pdMS_TO_TICKS(0x0CU))

/* Sensor. */
#define SENSOR_VAL (0x03U)
#define SENSOR_THRESHOLD_CENTRE_CHANNEL (100)
#define SENSOR_THRESHOLD_SIDE_CHANNEL   (90)

/* Angles. 
 * !!! DO NOT mark these macros as unsigned. !!! */
#define ANGLE_0    (0x00)
#define ANGLE_90   (90)
#define ANGLE_180  (180)
#define ANGLE_N180 (-180)
#define ANGLE_270  (270)
#define ANGLE_359  (359)
#define ANGLE_360  (360)
#define ANGLE_THRESHOLD (0x14)
#define OBSTACLE_ANGLE_THRESHOLD (0x0A)

/* Borders. */
#define BORDER_ALLOWANCE (45)
#define BORDER_THRESHOLD (445)
#define TARGET_THRESHOLD (10)

/* AWS. */
#define PARSE_ERR_THRESHOLD (0x0AU)

typedef enum
{
    TASK_PRIORITY_LOW = 0x01U,
    TASK_PRIORITY_MID = 0x05U,
    TASK_PRIORITY_HIGH = 0x0AU
} taskPriority_t;

typedef enum
{
    PHASE_0_TURN_CLOSEST_90_DEG,
    PHASE_1_TURN_NEXT_90_DEG,
    PHASE_2_MOVE_FWD
} obstacleAvoidancePhase_t;

typedef enum
{
    ROVER_DEF,
    ROVER_INIT,
    ROVER_OBSTACLE_DETECTED,
    ROVER_OBSTACLE_PROCESS,
    ROVER_BORDER_DETECTED,
    ROVER_BORDER_PROCESS,
    ROVER_FETCH_TGT,
    ROVER_MOVE_TO_TGT,
    ROVER_SUSPEND,
    /* This must always be the last state. */
    ROVER_NUM_OF_STATES
} roverStates_t;

/* Public Data Storage. */
static int16_t ROV_sensorVal[SENSOR_VAL] = {false};
static boolean ROV_obstacleDetected = false;
static int16_t ROV_roverX = 0x00;
static int16_t ROV_roverY = 0x00;
static int16_t ROV_roverDir = 0x00;
static int16_t ROV_targetX = 0x00;
static int16_t ROV_targetY = 0x00;
static int16_t ROV_turnAngle = 0x00;
static roverStates_t ROV_currRoverState = ROVER_DEF;
/* Currently, std::mutex does not support static initialization. */
std::mutex roverStateMutex;

/* PID.
 * Tuning result:
 * Proportional gain set to 3.2.
 * Integral gain unused.
 * Differential gain set to 0.1. */
double ROV_kP = 3.2;
double ROV_kI = 0x00;
double ROV_kD = 0.1;
double ROV_currDir = ROV_roverDir;
double ROV_desDir = ROV_turnAngle;
double ROV_corrTurnSpeed = 0x00;
PID turnPID (&ROV_currDir, &ROV_corrTurnSpeed, &ROV_desDir, ROV_kP, ROV_kI, ROV_kD, DIRECT);
double ROV_currX = ROV_roverX;
double ROV_desX = ROV_targetX;
double ROV_corrXSpeed = 0x00;
PID strXPID (&ROV_currX, &ROV_corrXSpeed, &ROV_desX, ROV_kP, ROV_kI, ROV_kD, DIRECT);
double ROV_currY = ROV_roverY;
double ROV_desY = ROV_targetY;
double ROV_corrYSpeed = 0x00;
PID strYPID (&ROV_currY, &ROV_corrYSpeed, &ROV_desY, ROV_kP, ROV_kI, ROV_kD, DIRECT);

/* Public Declarations. */
/* Task #1. */
void sensorControl (void *parameter);

/* Task #2. */
void AWSControl (void *parameter);
void parseAWSSubscription (void);
float getAbsoluteAngle (void);

/* Task #3. */
void roverControl (void *parameter);
void setMotorSpeedPIDControl (Direction motorDir, double corrSpeed);
void setManualMotorSpeedRamp (Direction motorDir, uint8_t finalMotorSpeed);
void determineMotorDirs (Direction roverDir, Direction *motorADir, Direction *motorBDir);
boolean isBorderWarningTriggered (void);

/* Task #4. */
void motorControl (void * parameter);

/* Public Definitions. */
/**
 * Name: setup.
 * Function: Handles SW and HW init.
 * Input: None.
 * Output: None.
 * Remarks: None.
 */
void setup (void)
{
    /* Set pinout for LED. */
    pinMode (LED_BOARD, OUTPUT);
    /* Set tx/rx baudrate. */
    Serial.begin (9600U);

    /* Init motors and sensors. */
    motorObject.SETUP();
    sensorObject.SETUP();
    /* Establish WiFi and AWS connection. */
    awsObject.connectAWS();

    /* Set up PID for motors. 
     * Mode is set to automatic. */
    turnPID.SetMode (AUTOMATIC);
    strXPID.SetMode (AUTOMATIC);
    strYPID.SetMode (AUTOMATIC);
    /* Refersh rate of PID controller. */
    turnPID.SetSampleTime (0x01);
    strXPID.SetSampleTime (0x01);
    strYPID.SetSampleTime (0x01);
    /* MAX PWM values for motors. */
    turnPID.SetOutputLimits(PID_MOTOR_MIN_TURN, PID_MOTOR_MAX_TURN);
    strXPID.SetOutputLimits(PID_MOTOR_MIN_FWD, PID_MOTOR_MAX_FWD);
    strYPID.SetOutputLimits(PID_MOTOR_MIN_FWD, PID_MOTOR_MAX_FWD);

    delay (1000U);

    /* Create task to monitor sensor values. */
    if (pdPASS == xTaskCreate (sensorControl, "ReadSensors", 2048U, NULL, TASK_PRIORITY_MID, NULL))
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Task #1 allocation successful.");
        #endif
    }
    else
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Task #1 allocation failed!");
        #endif
    }

    /* Create task to subscribe AWS topics and parse current co-ordinates. */
    if (pdPASS == xTaskCreate (AWSControl, "HandleAWS", 2048U, NULL, TASK_PRIORITY_MID, NULL))
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Task #2 allocation successful.");
        #endif
    }
    else
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Task #2 allocation failed!");
        #endif
    }

    /* Create task to control rover behavior. */
    if (pdPASS == xTaskCreate (roverControl, "MoveRoverToTargets", 4096U, NULL, TASK_PRIORITY_HIGH, NULL))
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Task #3 allocation successful.");
        #endif
    }
    else
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Task #3 allocation failed!");
        #endif
    }

    #ifdef ROVER_MANUAL_CONTROL
    /* Create task to control rover manually over wired serial comm. 
     * Note: This task is useless when rover is wireless. */
    if (pdPASS == xTaskCreate (motorControl, "RunMotors", 2048U, NULL, TASK_PRIORITY_LOW, NULL))
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Task #4 allocation successful.");
        #endif
    }
    else
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Task #4 allocation failed!");
        #endif
    }
    #endif

    #ifdef TASK_SCHEDULE_DEBUG
    Serial.println ("Actuator init complete.");
    #endif
}

/**
 * Name: loop.
 * Function: Background task.
 * Input: None.
 * Output: None.
 * Remarks: None.
 */
void loop (void)
{
    /* Nothing to do here .*/
}

/***************************************************
 ***************************************************
 ***********       SENSOR CONTROL        ***********
 ***************************************************
 **************************************************/
/**
 * Name: sensorControl.
 * Function: This task reads sensor values.
 * Input: No parameter is passed for now - NULL.
 * Output: None.
 * Remarks: Periodicity - 100ms.
 */
void sensorControl (void *parameter)
{
    int16_t *pSensor = NULL;

    while (true)
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Running task #1 - Sensor.");
        #endif
        /* Read sensor values. */
        pSensor = sensorObject.reading();

        for (uint8_t loopItr = 0x00U; loopItr < SENSOR_VAL; loopItr++)
        {
            ROV_sensorVal[loopItr] = *(pSensor + loopItr);
            #ifdef SENSOR_VAL_DEBUG
            Serial.printf ("%d\t", ROV_sensorVal[loopItr]);
            #endif
        }

        /* Indicate obstacle detection if an obstacle is detected. */
        if ((SENSOR_THRESHOLD_SIDE_CHANNEL > ROV_sensorVal[0x00U]) 
            || (SENSOR_THRESHOLD_CENTRE_CHANNEL > ROV_sensorVal[0x01U]) 
            || (SENSOR_THRESHOLD_SIDE_CHANNEL > ROV_sensorVal[0x02U]))
        {
            /* MArk obstacle detected. */
            ROV_obstacleDetected = true;
        }
        else
        {
            /* No obstacle detected. */
            ROV_obstacleDetected = false;
        }

        #ifdef SENSOR_VAL_DEBUG
        Serial.println ("");
        #endif

        vTaskDelay (DELAY_100);
    }

    /* Control will not reach here. */
    #ifdef TASK_SCHEDULE_DEBUG
    Serial.println ("Ending task #1.");
    #endif
    vTaskDelete (NULL);
}

/***************************************************
 ***************************************************
 ***********         HANDLE AWS          ***********
 ***************************************************
 **************************************************/
/**
 * Name: AWSControl.
 * Function: This task handles the AWS subscription and parses the subscribed data.
 * Input: No parameter is passed for now - NULL.
 * Output: None.
 * Remarks: Periodicity - 30ms.
 */
void AWSControl (void *parameter)
{
    while (true)
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Running task #2 - AWS.");
        #endif

        /* Wait for connection to be established before polling stayConnected. */
        if (awsObject.connectionEstablishedAWS)
        {
            /* Poll stayConnected every 30ms. */
            awsObject.stayConnected();

            /* Check if msg rcv flag is set to confirm fresh--
             * --reception to avoid duplicate computations. */
            if (awsObject.msgRcvIndicator)
            {
                /* Parse the subscribed messages to receive:
                 * 1. Rover X co-ordinate
                 * 2. Rover Y co-ordinate
                 * 3. Rover direction
                 * 4. Target X co-ordinate
                 * 5. Target Y co-ordinate. */
                parseAWSSubscription();

                /* Clear msg rcv flag to wait for fresh incoming msg. */
                awsObject.msgRcvIndicator = false;

                /* Compute the angle with which the rover must turn--
                 * --relative to the current target position. */
                ROV_turnAngle = getAbsoluteAngle();
            }
            else
            {
                /* Do not parse if fresh message hasn't been received. */
            }
            vTaskDelay (DELAY_30);
        }
        else
        {
            /* Do not poll stayConnected and parse data if WiFi/AWS connection is in progress. */
            vTaskDelay (DELAY_1000);
        }
    }

    /* Control will not reach here. */
    #ifdef TASK_SCHEDULE_DEBUG
    Serial.println("Ending task #2.");
    #endif
    vTaskDelete (NULL);
}

/**
 * Name: parseAWSSubscription.
 * Function: This function parses the message subscribed--
 *           --from interested AWS topics.
 * Input: None.
 * Output: None.
 * Remarks: None.
 */
void parseAWSSubscription (void)
{
    String subscriptionTopic;
    String subscriptionPayload;
    String roverPos;
    String targetPos;
    static uint8_t roverParseErr = 0x00U;

    subscriptionTopic = awsObject.subscriptionTopic;
    subscriptionPayload = awsObject.subscriptionPayload;

    /* Deserialize JSON dump. */
    StaticJsonDocument<200> docJSON;
    deserializeJson(docJSON, subscriptionPayload);

    if (subscriptionTopic == AWS_IOT_SUBSCRIBE_TOPIC_ROVER)
    {
        String roverPos = docJSON["rover"];

        /* Camera bug fix - do not decode roverPos if rover is not detected. */
        if (roverPos != "None")
        {
            /* Parse rover co-ordinates and direction from: {"rover": "{ID: [(X, Y), D]}"}. */
	        ROV_roverX = atoi (roverPos.substring(roverPos.indexOf('(') + 0x01U, (roverPos.indexOf(','))).c_str());
	        ROV_roverY = atoi (roverPos.substring(roverPos.indexOf(',') + 0x02U, (roverPos.indexOf(')'))).c_str());
	        ROV_roverDir = atoi (roverPos.substring(roverPos.indexOf(')') + 0x03U, (roverPos.indexOf(']'))).c_str());
            /* Reset error counter. */
            roverParseErr = 0x00U;
        }
        else
        {
            /* Increment counter to confirm error. */
            roverParseErr++;
            if (PARSE_ERR_THRESHOLD <= roverParseErr)
            {
                /* Force rover course correction. */
                ROV_roverX = 0x00;
                ROV_roverY = 0x00;
                ROV_roverDir = 0x00;
            }
            else
            {
                /* Do nothing. */
            }
        }

        #ifdef AWS_SUB_DEBUG
        Serial.printf ("Rover X: %d\n", ROV_roverX);
        Serial.printf ("Rover Y: %d\n", ROV_roverY);
        Serial.printf ("Rover Dir: %d\n", ROV_roverDir);
        #endif
    }
    else if (subscriptionTopic == AWS_IOT_SUBSCRIBE_TOPIC_TARGET)
    {
        String targetPos = docJSON["target"];

        /* targetPos will always be reliable unlike rover detection,--
         *--hence does not require error handling.
         * Parse target co-ordinates from: {"target": "(X, Y)"}. */
	    ROV_targetX = atoi (targetPos.substring(targetPos.indexOf('(') + 0x01U, targetPos.indexOf(',')).c_str());
	    ROV_targetY = atoi (targetPos.substring(targetPos.indexOf(',') + 0x02U, targetPos.indexOf(')')).c_str());

        #ifdef AWS_SUB_DEBUG
        Serial.printf ("Target X: %d\n", ROV_targetX);
        Serial.printf ("Target Y: %d\n", ROV_targetY);
        #endif
    }
    else
    {
        /* Take no action if topics don't match interest. */
    }
}

/**
 * Name: getAbsoluteAngle.
 * Function: This function computes the absolute angle the--
 *           --rover must turn to face the active target.
 * Input: None.
 * Output: The absolute angle to which the rover must turn.
 * Remarks: None.
 */
float getAbsoluteAngle (void)
{
    int16_t turnAngle = ANGLE_0;
    float invAngle;
   
    /* Rover and target match on X-axis. */
    if (ROV_roverX == ROV_targetX)
    {
        (ROV_roverY > ROV_targetY) ? turnAngle = ANGLE_90 : turnAngle = ANGLE_270;
    }
    /* Rover and target match on Y-axis. */
    else if (ROV_roverY == ROV_targetY)
    {
        (ROV_roverX > ROV_targetX) ? turnAngle = ANGLE_180 : turnAngle = ANGLE_0;
    }
    /* If neither axes match. */
    else
    {
        /* Slope in degrees. 
         * Degree = Radian * 180/pi. */
        invAngle = ((atan2((ROV_targetY - ROV_roverY), (ROV_targetX - ROV_roverX))) * (ANGLE_180 / PI));

        /* Angle: 0 - 180. */
        if ((ANGLE_0 < invAngle) && (ANGLE_180 >= invAngle))
        {
            turnAngle = ANGLE_360 - invAngle;
        }
        /* Angle: -180 - 0. */
        else if ((ANGLE_0 >= invAngle) && (ANGLE_N180 <= invAngle))
        {
            turnAngle = abs(invAngle);
        }   
        else
        {
            /* Do nothing. */
        }
    }
    
    return turnAngle;  
}

/***************************************************
 ***************************************************
 ***********        ROVER CONTROL        ***********
 ***************************************************
 **************************************************/
/**
 * Name: roverControl.
 * Function: This task controls the rover behavior--
 *           --to achieve targets.
 * Input: No parameter is passed for now - NULL.
 * Output: None.
 * Remarks: Periodicity - Dynamic based on state (10/30/100ms).
 */
void roverControl (void *parameter)
{
    uint16_t stateDelayTickTime = 0x64U;
    boolean isRoverTurning = false;
    int16_t obstacleAvoidanceAngle = ANGLE_360;
    obstacleAvoidancePhase_t obstacleAvoidancePhase = PHASE_0_TURN_CLOSEST_90_DEG;

    /* Wait for AWS connection before proceeding. */
    while (!awsObject.connectionEstablishedAWS)
    {
        vTaskDelay (pdMS_TO_TICKS(stateDelayTickTime));
    }

    /* Halt 5000ms before starting for debug. */
    stateDelayTickTime = DELAY_DEBUG;
    vTaskDelay (pdMS_TO_TICKS(stateDelayTickTime));

    while (true)
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Running task #3 - Rover.");
        #endif

        /* Overwrite current rover state based on external parameters.
        * Priority:
        * 1. Rover OP manual suspension
        * 2. Obstacle detection
        * 3. Border detection
        * 4. Target detection. */
        if (ROVER_SUSPEND != ROV_currRoverState)
        {
            roverStateMutex.lock();
            if (ROV_obstacleDetected)
            {
                /* Shift to obstacle detected only if rover is not already processing obstacle detection. */
                if ((ROVER_OBSTACLE_PROCESS != ROV_currRoverState) && (!isRoverTurning))
                {
                    ROV_currRoverState = ROVER_OBSTACLE_DETECTED;
                    #ifdef SENSOR_BORDER_TRIGGER_DEBUG
                    Serial.printf ("Obstacle triggered: %d, %d, %d\n", ROV_sensorVal[0], ROV_sensorVal[1], ROV_sensorVal[2]);
                    #endif
                }
                else
                {
                    /* Do not change current state. */
                }
            }
            else if (isBorderWarningTriggered())
            {
                /* Shift to border detected only if rover is not already processing border detection. */
                if ((ROVER_BORDER_PROCESS != ROV_currRoverState) && (!isRoverTurning))
                {
                    ROV_currRoverState = ROVER_BORDER_DETECTED;
                    #ifdef SENSOR_BORDER_TRIGGER_DEBUG
                    Serial.printf ("Border triggered: %d, %d, %d\n", ROV_roverX, ROV_roverY, ROV_roverDir);
                    #endif
                }
                else
                {
                    /* Do not change current state. */
                }
            }
            else
            {
                /* Do not change rover state without external interrupts. */
            }
            roverStateMutex.unlock();
        }
        else
        {
            /* Do nothing until rover operations are manually restored. */
        }

        /* Handle rover processing states to achieve targets. */
        switch (ROV_currRoverState)
        {
            /* State handles rover idle. 
             * Note: This state can be invoked manually by user from motorControl() task. */
            case ROVER_DEF:
                /* Indicate init using LED OFF. */
                digitalWrite (LED_BOARD, LOW);
                /* Do not break to allow roll-over to next state. */
           
            /* State handles rover init. 
             * State periodicity: 30ms. */ 
            case ROVER_INIT:
                roverStateMutex.lock();
                ROV_currRoverState = ROVER_FETCH_TGT;
                roverStateMutex.unlock();
                stateDelayTickTime = 0x1EU;
                break;

            /* State handles detection of obstacles: 
             * 1. Freeze all motor functions.
             * 2. Proceed to obstacle processing. 
             * State periodicity: 100ms. */
            case ROVER_OBSTACLE_DETECTED: 
                setManualMotorSpeedRamp (Forward, MOTOR_LOCK);
                roverStateMutex.lock();
                ROV_currRoverState = ROVER_OBSTACLE_PROCESS;
                roverStateMutex.unlock();
                stateDelayTickTime = 0x64U;
                break;

            /* State handles processing after obstacle detection: 
             * 1. Ascertain direction of turn.
             * 2. Move away from obstacle. 
             * State periodicity: 10-30ms. */
            case ROVER_OBSTACLE_PROCESS:
                /* First, we try to find the closest 90 degree--
                 * --turn possible with respect to the target.
                 * Quadrant 1: 0 - 90 deg. */
                if ((ANGLE_0 <= ROV_turnAngle)
                    && (ANGLE_90 > ROV_turnAngle))
                {
                    obstacleAvoidanceAngle = ANGLE_90;
                }
                /* Quadrant 2: 90 - 180 deg. */
                else if ((ANGLE_90 <= ROV_turnAngle)
                    && (ANGLE_180 > ROV_turnAngle))
                {
                    obstacleAvoidanceAngle = ANGLE_180;
                }
                /* Quadrant 3: 180 - 270 deg. */
                else if ((ANGLE_180 <= ROV_turnAngle)
                    && (ANGLE_270 > ROV_turnAngle))
                {
                    obstacleAvoidanceAngle = ANGLE_270;
                }
                /* Quadrant 4: 270 - 0 deg. */
                else if ((ANGLE_270 <= ROV_turnAngle)
                    && (ANGLE_360 > ROV_turnAngle))
                {
                    obstacleAvoidanceAngle = ANGLE_359;
                }
                else
                {
                    /* Invalid turn angle. */
                    obstacleAvoidanceAngle = ANGLE_359;
                }

                switch (obstacleAvoidancePhase)
                {
                    /* Check phase of obstacle detection. 
                     * PHASE 0: Turn towards determined direction of turn.
                     * Phase periodicity: 10ms. */
                    case PHASE_0_TURN_CLOSEST_90_DEG:
                        /* Turn until angle is achieved. */
                        if (OBSTACLE_ANGLE_THRESHOLD < abs(ROV_roverDir - obstacleAvoidanceAngle))
                        {
                            /* Determine direction of turn using angle difference between rover and target. */
                            if (ANGLE_0 > (ROV_roverDir - obstacleAvoidanceAngle))
                            {
                                setMotorSpeedPIDControl (Left, MOTOR_HIGH_TURN);
                            }
                            else
                            {
                                setMotorSpeedPIDControl (Right, MOTOR_HIGH_TURN);
                            }
                        }
                        /* Move to next state once angle is achieved. */
                        else
                        {
                            setManualMotorSpeedRamp (Forward, MOTOR_LOCK);
                            obstacleAvoidancePhase = PHASE_1_TURN_NEXT_90_DEG;
                        }
                        stateDelayTickTime = 0x0AU;
                        break;

                    /* PHASE 1: Check for obstacles in current angle of turn--
                     * --IF obstacle not present - shift to PHASE 2--
                     * --ELSE - determine other closest 90 degree turn--
                     * -- possible with respect to the target.
                     * Phase periodicity: 10ms. */
                    case PHASE_1_TURN_NEXT_90_DEG:
                        /* Check for obstacle in current angle. */
                        if (ROV_obstacleDetected)
                        {
                            /* If obstacle present, determine next closest--
                             * --90 degree w.r.t the target. */
                            obstacleAvoidanceAngle -= ANGLE_90;

                            /* Turn until angle is achieved. */
                            if (OBSTACLE_ANGLE_THRESHOLD < abs(ROV_roverDir - obstacleAvoidanceAngle))
                            {
                                /* Determine direction of turn using angle difference between rover and target. */
                                if (ANGLE_0 > (ROV_roverDir - obstacleAvoidanceAngle))
                                {
                                    setMotorSpeedPIDControl (Left, MOTOR_HIGH_TURN);
                                }
                                else
                                {
                                    setMotorSpeedPIDControl (Right, MOTOR_HIGH_TURN);
                                }
                            }
                            /* Move to next state once angle is achieved. */
                            else
                            {
                                setManualMotorSpeedRamp (Forward, MOTOR_LOCK);
                                obstacleAvoidancePhase = PHASE_2_MOVE_FWD;
                            }
                        }
                        else
                        {
                            /* If no obstacle, proceed to PHASE 2. */
                            obstacleAvoidancePhase = PHASE_2_MOVE_FWD;
                        }
                        stateDelayTickTime = 0x0AU;
                        break;
                
                    /* PHASE 2: Move forward for 3 seconds.
                     * Phase periodicity: 30ms. */
                    case PHASE_2_MOVE_FWD:
                        obstacleAvoidancePhase = PHASE_0_TURN_CLOSEST_90_DEG;
                        setManualMotorSpeedRamp (Forward, MOTOR_HIGH_FWD);
                        /* This allows the rover to move fwd while processing other tasks. */
                        vTaskDelay (DELAY_3000);
                        roverStateMutex.lock();
                        ROV_currRoverState = ROVER_FETCH_TGT;
                        roverStateMutex.unlock();
                        stateDelayTickTime = 0x1EU;

                    default:
                        /* Do nothing. */
                        break;
                }
                break;

            /* State handles detection of border: 
             * 1. Freeze motor functions.
             * 2. Proceed to border processing. 
             * State periodicity: 100ms. */
            case ROVER_BORDER_DETECTED:
                setManualMotorSpeedRamp (Forward, MOTOR_LOCK);
                roverStateMutex.lock();
                ROV_currRoverState = ROVER_BORDER_PROCESS;
                roverStateMutex.unlock();
                stateDelayTickTime = 0x64U;
                break;

            /* State handles processing after obstacle detection: 
             * 1. Blind reverse. 
             * State periodicity: 30ms. */
            case ROVER_BORDER_PROCESS:
                setManualMotorSpeedRamp (Backward, MOTOR_HIGH_FWD);
                vTaskDelay (DELAY_200);
                roverStateMutex.lock();
                ROV_currRoverState = ROVER_FETCH_TGT;
                roverStateMutex.unlock();
                stateDelayTickTime = 0x1EU;
                break;

            /* State handles fetching target co-ordinates. 
             * Note: Dummy state - this is managed in the AWSControl() task. */
            case ROVER_FETCH_TGT:
                /* Do nothing. 
                 * Do not break to allow roll-over to next state. */

            /* State handles moving to target: 
             * 1. Ascertain position of target.
             * 2. Move to target. 
             * State periodicity: 10ms. */
            case ROVER_MOVE_TO_TGT:
                /* Assign fresh values continually for PID. */
                ROV_currDir = ROV_roverDir;
                ROV_desDir = ROV_turnAngle;
                ROV_currX = ROV_roverX;
                ROV_desX = ROV_targetX;
                ROV_currY = ROV_roverY;
                ROV_desY = ROV_targetY;
        
                /* Compute new PID output. */
                turnPID.Compute();
                strXPID.Compute();
                strYPID.Compute();

                /* This section handles the algorithm to move rover to target. */
                if (ANGLE_THRESHOLD < abs(ROV_roverDir - ROV_turnAngle))
                {
                    /* Determine direction of turn using angle difference between rover and target. 
                     * Turning is done using PID to compensate for camera lag and angle overshoot. 
                     * We do not let the PID algorithm decide on the direction of turn here--
                     * --as it is often indecisive! */
                    if (ANGLE_0 > (ROV_roverDir - ROV_turnAngle))
                    {
                        setMotorSpeedPIDControl (Left, ROV_corrTurnSpeed);
                    }
                    else
                    {
                        setMotorSpeedPIDControl (Right, ROV_corrTurnSpeed);
                    }
                    isRoverTurning = true;
                }
                else
                {
                    /* Halt if target is achieved! */
                    if ((TARGET_THRESHOLD >= abs(ROV_targetX - ROV_roverX)) 
                        && (TARGET_THRESHOLD >= abs(ROV_targetY - ROV_roverY)))
                    {
                        setManualMotorSpeedRamp (Forward, MOTOR_LOCK);
                    }
                    /* Proceed geradaus to target. */
                    else
                    {
                        setMotorSpeedPIDControl (Forward, (((abs(ROV_corrXSpeed)) + (abs(ROV_corrYSpeed))) / 0x02U));
                    }
                    isRoverTurning = false;
                }
                stateDelayTickTime = 0x0AU;
                break;

            /* State suspends all operations.
             * Note: This state is invoked manually by user from motorControl() task. 
             State periodicity: 100ms. */
            case ROVER_SUSPEND:
                /* Cease all functionality.
                 * Indicate suspension using LED ON. */
                digitalWrite (LED_BOARD, HIGH);
                stateDelayTickTime = 0x64U;
                break;

            default:
                /* Do nothing. */
                break;
        }
        /* Dynamically allocated periodicity based on current state. */
        vTaskDelay (pdMS_TO_TICKS(stateDelayTickTime));
    }

    /* Control will not reach here. */
    #ifdef TASK_SCHEDULE_DEBUG
    Serial.println ("Ending task #3.");
    #endif
    vTaskDelete (NULL);
}

/**
 * Name: setMotorSpeedPIDControl.
 * Function: This function computes the individual motor--
 *           --speeds from the PID library output.
 * Input: Desired speed, required speed correction.
 * Output: None.
 * Remarks: None.
 */
void setMotorSpeedPIDControl (Direction motorDir, double corrSpeed)
{
    Direction motorADir;
    Direction motorBDir;

    /* Determine motor A and B direction based on intended rover direction. */
    determineMotorDirs (motorDir, &motorADir, &motorBDir);

    motorObject.set_speed (MotorA, motorADir, abs(corrSpeed));
    motorObject.set_speed (MotorB, motorBDir, abs(corrSpeed));

    //Serial.printf ("PID: %d, %d\n", motorDir, corrSpeed);

    #ifdef MOTOR_SPEED_DEBUG
    Serial.printf ("PID motor direction: A: %s, B: %s\n", 
                   motorADir ? "Backward" : "Forward", motorBDir ? "Backward" : "Forward");
    Serial.printf ("PID motor speed: %d\n", corrSpeed);
    #endif
}

/**
 * Name: setManualMotorSpeedRamp.
 * Function: This function achieves intended motor speed linearly.
 * Input: Motor A direction, motor B direction, final motor speed.
 * Output: None.
 * Remarks: None.
 */
void setManualMotorSpeedRamp (Direction roverDir, uint8_t finalMotorSpeed)
{
    static uint8_t prevSpeed = MOTOR_LOCK;
    static Direction prevMotorADir = Forward;
    static Direction prevMotorBDir = Forward;
    Direction motorADir;
    Direction motorBDir;

    /* Determine motor A and B direction based on intended rover direction. */
    determineMotorDirs (roverDir, &motorADir, &motorBDir);

    #ifdef MOTOR_SPEED_DEBUG
    Serial.printf("Previous motor speed: %d\n", prevSpeed);
    Serial.printf("Previous motor directions: A: %s, B: %s\n", 
                   prevMotorADir ? "Backward" : "Forward", prevMotorBDir ? "Backward" : "Forward");
    #endif
    /* WARNING: Do not reduce size/type - endless loop will trigger WDT. */
    int16_t loopItr = 0x00;

    /* If requested speed is equal to current speed. */
    if (prevSpeed == finalMotorSpeed)
    {
        /* If motor direction needs to be changed. */
        if ((prevMotorADir != motorADir) || (prevMotorBDir != motorBDir))
        {
            /* Decrease speed to LOW. */
            #ifdef LINEAR_SPEED_REDUCTION
            /* Linear deceleration. */
            for (loopItr = finalMotorSpeed; loopItr >= MOTOR_LOW; loopItr -= MOTOR_STEP_SIZE)
            {
                #ifdef MOTOR_SPEED_DEBUG
                Serial.printf("Motor speed: %d\n", loopItr);
                #endif
                motorObject.set_speed (MotorA, prevMotorADir, loopItr);
                motorObject.set_speed (MotorB, prevMotorBDir, loopItr);
                vTaskDelay (MOTOR_STEP_DELAY);
            }
            #else
            /* Instantaneous deceleration - mapped to slide 49 of ES Lab Project-Detailed Task Description.HTML. */
            motorObject.set_speed (MotorA, motorADir, MOTOR_LOW);
            motorObject.set_speed (MotorB, motorBDir, MOTOR_LOW);
            #ifdef MOTOR_SPEED_DEBUG
            Serial.printf("Motor speed: %d\n", MOTOR_LOW);
            #endif
            #endif
            #ifdef MOTOR_SPEED_DEBUG
            Serial.printf("Switching motor direction to: A: %s, B: %s\n", 
                           motorADir ? "Backward" : "Forward", motorBDir ? "Backward" : "Forward");
            #endif
            /* Increase speed to requested speed with switch in motor direction. */
            for (loopItr = MOTOR_LOW + MOTOR_STEP_SIZE; loopItr <= finalMotorSpeed; loopItr += MOTOR_STEP_SIZE)
            {
                #ifdef MOTOR_SPEED_DEBUG
                Serial.printf("Motor speed: %d\n", loopItr);
                #endif
                motorObject.set_speed (MotorA, motorADir, loopItr);
                motorObject.set_speed (MotorB, motorBDir, loopItr);
                vTaskDelay (MOTOR_STEP_DELAY);
            }
        }
        /* If motor direction need not to be changed - duplicate instruction. */
        else
        {
            motorObject.set_speed (MotorA, motorADir, finalMotorSpeed);
            motorObject.set_speed (MotorB, motorBDir, finalMotorSpeed);
        }
    }
    /* If requested speed is greater than current speed. */
    else if (prevSpeed < finalMotorSpeed)
    {
        for (loopItr = prevSpeed; loopItr <= finalMotorSpeed; loopItr += MOTOR_STEP_SIZE)
        {
            #ifdef MOTOR_SPEED_DEBUG
            Serial.printf("Motor speed: %d\n", loopItr);
            #endif
            motorObject.set_speed (MotorA, motorADir, loopItr);
            motorObject.set_speed (MotorB, motorBDir, loopItr);
            vTaskDelay (MOTOR_STEP_DELAY);
        }
    }
    /* If requested speed is lower than current speed. */
    else
    {
        #ifdef LINEAR_SPEED_REDUCTION
        /* Linear deceleration. */
        for (loopItr = prevSpeed; loopItr >= finalMotorSpeed; (loopItr -= MOTOR_STEP_SIZE))
        {
            #ifdef MOTOR_SPEED_DEBUG
            Serial.printf("Motor speed: %d\n", loopItr);
            #endif
            motorObject.set_speed (MotorA, motorADir, loopItr);
            motorObject.set_speed (MotorB, motorBDir, loopItr);
            vTaskDelay (MOTOR_STEP_DELAY);
        }
        #else
        /* Instantaneous deceleration - mapped to slide 49 of ES Lab Project-Detailed Task Description.HTML. */
        motorObject.set_speed (MotorA, motorADir, finalMotorSpeed);
        motorObject.set_speed (MotorB, motorBDir, finalMotorSpeed);
        #endif
    }

    prevSpeed = finalMotorSpeed;
    prevMotorADir = motorADir;
    prevMotorBDir = motorBDir;
    #ifdef MOTOR_SPEED_DEBUG
    Serial.printf("Final motor speed: %d\n", finalMotorSpeed);
    Serial.printf("Final motor directions: A: %s, B: %s\n", 
                   motorADir ? "Backward" : "Forward", motorBDir ? "Backward" : "Forward");
    #endif
}

/**
 * Name: determineMotorDirs.
 * Function: This function determines motor A and motor B directions--
 *           --from intended rover direction.
 * Input: Intended rover direction, pointer to motor A direction,--
 *        --pointer to motor B direction.
 * Output: None.
 * Remarks: None.
 */
void determineMotorDirs (Direction roverDir, Direction *motorADir, Direction *motorBDir)
{
    /* Determine motor A and B direction based on intended rover direction. */
    if (Forward == roverDir)
    {
        *motorADir = Forward;
        *motorBDir = Forward;
    }
    else if (Backward == roverDir)
    {
        *motorADir = Backward;
        *motorBDir = Backward;
    }
    /* Left: motorA-Bwd, motorB-Fwd. */
    else if (Left == roverDir)
    {
        *motorADir = Backward;
        *motorBDir = Forward;
    }
    /* Right: motorA-Fwd, motorB-Bwd. */
    else if (Right == roverDir)
    {
        *motorADir = Forward;
        *motorBDir = Backward;
    }
    else
    {
        /* No assignment - invalid condition. */
    }
}

/**
 * Name: isBorderWarningTriggered.
 * Function: This function computes if current rover-
 *           -co-ordinates are close to and facing borders.
 * Input: None.
 * Output: Boolean if rover is close to border.
 * Remarks: None.
 */
boolean isBorderWarningTriggered (void)
{
    boolean retVal = false;

    /* Quadrant 1: 0 - 90 deg. */
    if ((ANGLE_0 <= ROV_roverDir) 
        && (ANGLE_90 > ROV_roverDir))
    {
        /* Borders: Right + Top. */
        if ((BORDER_ALLOWANCE >= (abs(ROV_roverX - BORDER_THRESHOLD))) 
            || (BORDER_ALLOWANCE >= ROV_roverY))
        {
            retVal = true;
        }
        else
        {
            /* Do nothing. */
        }
    }
    /* Quadrant 2: 90 - 180 deg. */
    else if ((ANGLE_90 <= ROV_roverDir) 
        && (ANGLE_180 > ROV_roverDir))
    {
        /* Borders: Top + Left. */
        if ((BORDER_ALLOWANCE >= ROV_roverY) 
            || (BORDER_ALLOWANCE >= ROV_roverX))
        {
            retVal = true;
        }
        else
        {
            /* Do nothing. */
        }
    }
    /* Quadrant 3: 180 - 270 deg. */
    else if ((ANGLE_180 <= ROV_roverDir) 
        && (ANGLE_270 > ROV_roverDir))
    {
        /* Borders: Left + Bottom. */
        if ((BORDER_ALLOWANCE >= ROV_roverX) 
            || (BORDER_ALLOWANCE >= abs(ROV_roverY - BORDER_THRESHOLD)))
        {
            retVal = true;
        }
        else
        {
            /* Do nothing. */
        }
    }
    /* Quadrant 4: 270 - 0 deg. */
    else if ((ANGLE_270 <= ROV_roverDir) 
        && (ANGLE_360 > ROV_roverDir))
    {
        /* Borders: Bottom + Right. */
        if ((BORDER_ALLOWANCE >= (abs(ROV_roverY - BORDER_THRESHOLD))) 
            || (BORDER_ALLOWANCE >= (abs(ROV_roverX - BORDER_THRESHOLD))))
        {
            retVal = true;
        }
        else
        {
            /* Do nothing - invalid turn angle. */
        }
    }
    else
    {
        /* Error - control must not reach here. */
        retVal = true;
    }
    
    return retVal;
}

#ifdef ROVER_MANUAL_CONTROL
/***************************************************
 ***************************************************
 ***********        MOTOR CONTROL        ***********
 ***************************************************
 **************************************************/
/**
 * Name: motorControl.
 * Function: This task allows user to control the motors--
 *           --manually using keyboard inputs.
 * Input: No parameter is passed for now - NULL.
 * Output: None.
 * Remarks: Periodicity - 500ms.
 */
void motorControl (void * parameter)
{
    char controlCmd;
  
    while (true)
    {
        #ifdef TASK_SCHEDULE_DEBUG
        Serial.println ("Running task #4 - Motor.");
        #endif

        /* Detect user input to control motor direction. 
         * Max speed is enabled here since the directions are--
         * --user-controlled. */
        if (false < Serial.available())
        {
            controlCmd = Serial.read();
            switch (controlCmd)
            {
                /* Forward. */
                case 'w':
                case 'W':
                    setManualMotorSpeedRamp (Forward, MOTOR_HIGH_FWD);
                    break;

                /* Backward. */
                case 's':
                case 'S':
                    setManualMotorSpeedRamp (Backward, MOTOR_HIGH_FWD);
                    break;

                /* Left. */
                case 'a':
                case 'A':
                    setManualMotorSpeedRamp (Left, MOTOR_HIGH_TURN);
                    break;

                /* Right. */
                case 'd':
                case 'D':
                    setManualMotorSpeedRamp (Right, MOTOR_HIGH_TURN);
                    break;

                /* Stop. */
                case 'q':
                case 'Q':
                    setManualMotorSpeedRamp (Forward, MOTOR_LOCK);
                    break;

                /* Reset rover - re-init state machine. */
                case 'o':
                case 'O':
                    roverStateMutex.lock();
                    ROV_currRoverState = ROVER_DEF;
                    roverStateMutex.unlock();
                    break;

                /* Pause rover - suspend state machine. */
                case 'p':
                case 'P':
                    /* Freeze all motor functions. */
                    setManualMotorSpeedRamp (Forward, MOTOR_LOCK);
                    roverStateMutex.lock();
                    ROV_currRoverState = ROVER_SUSPEND;
                    roverStateMutex.unlock();
                    break;

                default:
                    /* Do nothing. */
                    break;
            }
        }

        vTaskDelay (DELAY_500);
    }

    /* Control will not reach here. */
    #ifdef TASK_SCHEDULE_DEBUG
    Serial.println("Ending task #4.");
    #endif
    vTaskDelete (NULL);
}
#endif

