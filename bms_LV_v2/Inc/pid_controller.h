//*********************************************************************************
// Daniele Facinelli   Eagle-TRT  27/04/18
//
// PID library for Pumps and fan control of cooling system developed for Chimera Evo
//
//
//*********************************************************************************

//
// Header Guard
//
#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

//*********************************************************************************
// Headers
//*********************************************************************************
#include <stdint.h>
#include <stdbool.h>

//*********************************************************************************
// Macros and Globals
//*********************************************************************************

typedef enum
{
    MANUAL,
    AUTOMATIC
} PIDMode;

typedef enum
{
    DIRECT,
    REVERSE
} PIDDirection;

typedef struct
{
    //
    // Input to the PID Controller
    //
    float input;

    //
    // Previous input to the PID Controller
    //
    float lastInput;

    //
    // Output of the PID Controller
    //
    float output;

    //
    // Gain constant values that were passed by the user
    // These are for display purposes
    //
    float dispKp;
    float dispKi;
    float dispKd;

    //
    // Gain constant values that the controller alters for
    // its own use
    //
    float alteredKp;
    float alteredKi;
    float alteredKd;

    //
    // The Integral Term
    //
    float iTerm;

    //
    // The interval (in seconds) on which the PID controller
    // will be called
    //
    float sampleTime;

    //
    // The values that the output will be constrained to
    //
    float outMin;
    float outMax;

    //
    // The user chosen operating point
    //
    float setpoint;

    //
    // The sense of direction of the controller
    // DIRECT:  A positive setpoint gives a positive output
    // REVERSE: A positive setpoint gives a negative output
    //
    PIDDirection controllerDirection;

    //
    // Tells how the controller should respond if the user has
    // taken over manual control or not
    // MANUAL:    PID controller is off.
    // AUTOMATIC: PID controller is on.
    //
    PIDMode mode;
} PIDControl;

//*********************************************************************************
// Prototypes
//*********************************************************************************

//
// PID Initialize
// Description:
//      Initializes a PIDControl instantiation. This should be called at least once
//      before any other PID functions are called.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      kp - Positive P gain constant value.
//      ki - Positive I gain constant value.
//      kd - Positive D gain constant value.
//      sampleTimeSeconds - Interval in seconds on which PIDCompute will be called.
//      minOutput - Constrain PID output to this minimum value.
//      maxOutput - Constrain PID output to this maximum value.
//      mode - Tells how the controller should respond if the user has taken over
//             manual control or not.
//             MANUAL:    PID controller is off. User can manually control the
//                        output.
//             AUTOMATIC: PID controller is on. PID controller controls the output.
//      controllerDirection - The sense of direction of the controller
//                            DIRECT:  A positive setpoint gives a positive output.
//                            REVERSE: A positive setpoint gives a negative output.
// Returns:
//      Nothing.
//
extern void PIDInit(PIDControl *pid, float kp, float ki, float kd,
                    float sampleTimeSeconds, float minOutput, float maxOutput,
                    PIDMode mode, PIDDirection controllerDirection);

//
// PID Compute
// Description:
//      Should be called on a regular interval defined by sampleTimeSeconds.
//      Typically, PIDSetpointSet and PIDInputSet should be called immediately
//      before PIDCompute.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      True if in AUTOMATIC. False if in MANUAL.
//
extern bool PIDCompute(PIDControl *pid);

//
// PID Mode Set
// Description:
//      Sets the PID controller to a new mode.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      mode - Tells how the controller should respond if the user has taken over
//             manual control or not.
//             MANUAL:    PID controller is off. User can manually control the
//                        output.
//             AUTOMATIC: PID controller is on. PID controller controls the output.
// Returns:
//      Nothing.
//
extern void PIDModeSet(PIDControl *pid, PIDMode mode);

//
// PID Output Limits Set
// Description:
//      Sets the new output limits. The new limits are applied to the PID
//      immediately.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      min - Constrain PID output to this minimum value.
//      max - Constrain PID output to this maximum value.
// Returns:
//      Nothing.
//
extern void PIDOutputLimitsSet(PIDControl *pid, float min, float max);

//
// PID Tunings Set
// Description:
//      Sets the new gain constant values.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      kp - Positive P gain constant value.
//      ki - Positive I gain constant value.
//      kd - Positive D gain constant value.
// Returns:
//      Nothing.
//
extern void PIDTuningsSet(PIDControl *pid, float kp, float ki, float kd);

//
// PID Tuning Gain Constant P Set
// Description:
//      Sets the proportional gain constant value.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      kp - Positive P gain constant value.
// Returns:
//      Nothing.
//
extern void PIDTuningKpSet(PIDControl *pid, float kp);

//
// PID Tuning Gain Constant I Set
// Description:
//      Sets the proportional gain constant value.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      ki - Positive I gain constant value.
// Returns:
//      Nothing.
//
extern void PIDTuningKiSet(PIDControl *pid, float ki);

//
// PID Tuning Gain Constant D Set
// Description:
//      Sets the proportional gain constant value.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      kd - Positive D gain constant value.
// Returns:
//      Nothing.
//
extern void PIDTuningKdSet(PIDControl *pid, float kd);

//
// PID Controller Direction Set
// Description:
//      Sets the new controller direction.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      controllerDirection - The sense of direction of the controller
//                            DIRECT:  A positive setpoint gives a positive output
//                            REVERSE: A positive setpoint gives a negative output
// Returns:
//      Nothing.
//
extern void PIDControllerDirectionSet(PIDControl *pid,
                                      PIDDirection controllerDirection);

//
// PID Sample Time Set
// Description:
//      Sets the new sampling time (in seconds).
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      sampleTimeSeconds - Interval in seconds on which PIDCompute will be called.
// Returns:
//      Nothing.
//
extern void PIDSampleTimeSet(PIDControl *pid, float sampleTimeSeconds);

//
// Basic Set and Get Functions for PID Parameters
//

//
// PID Setpoint Set
// Description:
//      Alters the setpoint the PID controller will try to achieve.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      setpoint - The desired setpoint the PID controller will try to obtain.
// Returns:
//      Nothing.
//
inline void
PIDSetpointSet(PIDControl *pid, float setpoint) { pid->setpoint = setpoint; }

//
// PID Input Set
// Description:
//      Should be called before calling PIDCompute so the PID controller will
//      have an updated input value to work with.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      input - The value the controller will work with.
// Returns:
//      Nothing.
//
inline void
PIDInputSet(PIDControl *pid, float input) { pid->input = input; }

//
// PID Output Get
// Description:
//      Typically, this function is called after PIDCompute in order to
//      retrieve the output of the controller.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      The output of the specific PID controller.
//
inline float
PIDOutputGet(PIDControl *pid) { return pid->output; }

//
// PID Proportional Gain Constant Get
// Description:
//      Returns the proportional gain constant value the particular
//      controller is set to.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      The proportional gain constant.
//
inline float
PIDKpGet(PIDControl *pid) { return pid->dispKp; }

//
// PID Integral Gain Constant Get
// Description:
//      Returns the integral gain constant value the particular
//      controller is set to.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      The integral gain constant.
//
inline float
PIDKiGet(PIDControl *pid) { return pid->dispKi; }

//
// PID Derivative Gain Constant Get
// Description:
//      Returns the derivative gain constant value the particular
//      controller is set to.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      The derivative gain constant.
//
inline float
PIDKdGet(PIDControl *pid) { return pid->dispKd; }

//
// PID Mode Get
// Description:
//      Returns the mode the particular controller is set to.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      MANUAL or AUTOMATIC depending on what the user set the
//      controller to.
//
inline PIDMode
PIDModeGet(PIDControl *pid) { return pid->mode; }

//
// PID Direction Get
// Description:
//      Returns the direction the particular controller is set to.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      DIRECT or REVERSE depending on what the user set the
//      controller to.
//
inline PIDDirection
PIDDirectionGet(PIDControl *pid) { return pid->controllerDirection; }

inline float
PIDInputGet(PIDControl *pid) { return pid->input; }

#endif // PID_CONTROLLER_H
