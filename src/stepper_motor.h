#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

/*
 * Stepper Motor Control via FastAccelStepper
 *
 * Registers three agent functions:
 *   - stepper_move : move absolute or relative; sign of steps sets direction;
 *                    speed and acceleration are optional per-call overrides
 *   - stepper_stop : stop with deceleration or immediate force-stop
 *   - get_stepper_state : poll position & running state (call regularly,
 *                         same pattern as registerSensorFunctionUpdated)
 *
 * Adjust the pin constants below to match your wiring before use.
 */

#include "agent.h"
#include "port_builder.h"
#include <FastAccelStepper.h>

// ── Pin configuration – adjust to match your hardware ────────────────────────
#define STEPPER_STEP_PIN   4  // GPIO connected to driver STEP input
#define STEPPER_DIR_PIN    5  // GPIO connected to driver DIR  input
#define STEPPER_ENABLE_PIN 6  // GPIO connected to driver EN   input; set to -1 to skip
// ─────────────────────────────────────────────────────────────────────────────

// Persistent JSON documents (must outlive the agent for the lifetime of the program)
DynamicJsonDocument stepperMoveArgsDoc(512);
DynamicJsonDocument stepperMoveReturnsDoc(256);
DynamicJsonDocument stepperStopArgsDoc(256);
DynamicJsonDocument stepperStopReturnsDoc(256);
DynamicJsonDocument stepperStateReturnsDoc(512);

// Global stepper engine and motor handle
static FastAccelStepperEngine stepperEngine;
static FastAccelStepper *stepper = nullptr;
static bool stepperInitialized = false;

// Initialize the FastAccelStepper engine and bind to the configured pins
static void initStepper()
{
    stepperEngine.init();
    stepper = stepperEngine.stepperConnectToPin(STEPPER_STEP_PIN);

    if (stepper == nullptr)
    {
        Serial.println("✗ Stepper: failed to connect to step pin " + String(STEPPER_STEP_PIN));
        return;
    }

    stepper->setDirectionPin(STEPPER_DIR_PIN);

    if (STEPPER_ENABLE_PIN >= 0)
    {
        stepper->setEnablePin(STEPPER_ENABLE_PIN);
        stepper->setAutoEnable(true);
    }

    // Sensible power-on defaults
    stepper->setSpeedInHz(1000);
    stepper->setAcceleration(500);

    stepperInitialized = true;
    Serial.println("✓ Stepper initialized  step=" + String(STEPPER_STEP_PIN) +
                   "  dir=" + String(STEPPER_DIR_PIN) +
                   "  en=" + String(STEPPER_ENABLE_PIN));
}

// ── stepper_move ──────────────────────────────────────────────────────────────
void registerStepperMove(Agent *agent)
{
    FunctionDefinition def = DefinitionBuilder::create(
        "stepper_move",
        "Move the stepper motor. Positive steps move forward, negative steps move backward. "
        "Use mode='absolute' to target an absolute position or 'relative' for a step offset.",
        "FUNCTION",
        false);

    JsonArray args = stepperMoveArgsDoc.to<JsonArray>();

    JsonObject stepsArg = PortBuilder::createIntPort(
        args, "steps", "Steps / Target",
        "Steps to move (relative) or target position (absolute). Sign sets direction.", false);
    PortBuilder::setDefault(stepsArg, 0);

    JsonObject modeArg = PortBuilder::createStringPort(
        args, "mode", "Mode",
        "absolute: go to exact step position; relative: move by this many steps from current", false);
    PortBuilder::addChoice(modeArg, "Relative", "relative", "Move by the given number of steps");
    PortBuilder::addChoice(modeArg, "Absolute", "absolute", "Move to the given absolute position");
    PortBuilder::addChoiceWidget(modeArg);
    PortBuilder::setDefault(modeArg, "relative");

    JsonObject speedArg = PortBuilder::createIntPort(
        args, "speed_hz", "Speed (steps/s)",
        "Motor speed in steps per second (optional, keeps last value if omitted)", true);
    PortBuilder::setDefault(speedArg, 1000);
    PortBuilder::addSliderWidget(speedArg, 1, 10000, 100);

    JsonObject accelArg = PortBuilder::createIntPort(
        args, "acceleration", "Acceleration (steps/s²)",
        "Ramp acceleration in steps per second squared (optional, keeps last value if omitted)", true);
    PortBuilder::setDefault(accelArg, 500);
    PortBuilder::addSliderWidget(accelArg, 1, 5000, 100);

    def.args = args;

    JsonArray returns = stepperMoveReturnsDoc.to<JsonArray>();
    PortBuilder::createIntPort(returns, "current_position", "Current Position",
                               "Step position at the time the command was issued", false);
    PortBuilder::createIntPort(returns, "target_position", "Target Position",
                               "Destination step position", false);
    PortBuilder::createBoolPort(returns, "success", "Success",
                                "Whether the move command was accepted", false);

    def.returns = returns;

    agent->registerFunction(
        "stepper_move",
        def,
        [](Agent &agent, JsonObject args, JsonObject &returns) -> bool
        {
            if (!stepperInitialized || stepper == nullptr)
            {
                Serial.println("✗ stepper_move: stepper not initialized");
                returns["success"] = false;
                return false;
            }

            int32_t steps = (int32_t)(args["steps"] | 0);
            String mode = args["mode"] | "relative";

            // Apply speed/acceleration only when the caller supplies them
            if (!args["speed_hz"].isNull())
            {
                uint32_t speedHz = (uint32_t)max(1, (int)(args["speed_hz"] | 1000));
                stepper->setSpeedInHz(speedHz);
            }

            if (!args["acceleration"].isNull())
            {
                uint32_t accel = (uint32_t)max(1, (int)(args["acceleration"] | 500));
                stepper->setAcceleration(accel);
            }

            if (mode == "absolute")
            {
                stepper->moveTo(steps);
            }
            else
            {
                stepper->move(steps);
            }

            returns["current_position"] = (int32_t)stepper->getCurrentPosition();
            returns["target_position"] = (int32_t)stepper->getPositionAfterCommandsCompleted();
            returns["success"] = true;

            Serial.println("stepper_move  mode=" + mode +
                           "  steps=" + String(steps) +
                           "  pos=" + String(stepper->getCurrentPosition()));
            return true;
        });
}

// ── stepper_stop ──────────────────────────────────────────────────────────────
void registerStepperStop(Agent *agent)
{
    FunctionDefinition def = DefinitionBuilder::create(
        "stepper_stop",
        "Stop the stepper motor. Use emergency=true for an immediate halt, "
        "or false (default) to decelerate smoothly to a stop.",
        "FUNCTION",
        false);

    JsonArray args = stepperStopArgsDoc.to<JsonArray>();

    JsonObject emergencyArg = PortBuilder::createBoolPort(
        args, "emergency", "Emergency Stop",
        "true = immediate stop (no deceleration); false = ramp down smoothly", false);
    PortBuilder::setDefault(emergencyArg, false);

    def.args = args;

    JsonArray returns = stepperStopReturnsDoc.to<JsonArray>();
    PortBuilder::createIntPort(returns, "position", "Position",
                               "Step position when the stop command was issued", false);
    PortBuilder::createBoolPort(returns, "success", "Success",
                                "Whether the stop command was issued", false);

    def.returns = returns;

    agent->registerFunction(
        "stepper_stop",
        def,
        [](Agent &agent, JsonObject args, JsonObject &returns) -> bool
        {
            if (!stepperInitialized || stepper == nullptr)
            {
                returns["success"] = false;
                return false;
            }

            bool emergency = args["emergency"] | false;

            if (emergency)
            {
                stepper->forceStop();
            }
            else
            {
                stepper->stopMove();
            }

            returns["position"] = (int32_t)stepper->getCurrentPosition();
            returns["success"] = true;

            Serial.println(String("stepper_stop") + (emergency ? " (emergency)" : " (decel)") +
                           "  pos=" + String(stepper->getCurrentPosition()));
            return true;
        });
}

// ── get_stepper_state ─────────────────────────────────────────────────────────
// No arguments, mirrors the sensor pattern: call this regularly to poll motor state.
void registerStepperGetState(Agent *agent)
{
    FunctionDefinition def = DefinitionBuilder::create(
        "get_stepper_state",
        "Read the current stepper motor position and running state. "
        "Call this regularly to track position, similar to a sensor reading.",
        "FUNCTION",
        false);

    // No input arguments
    JsonArray args;
    def.args = args;

    JsonArray returns = stepperStateReturnsDoc.to<JsonArray>();
    PortBuilder::createIntPort(returns, "current_position", "Current Position",
                               "Current step position", false);
    PortBuilder::createIntPort(returns, "target_position", "Target Position",
                               "Target step position (equals current_position when idle)", false);
    PortBuilder::createBoolPort(returns, "is_running", "Is Running",
                                "True while the motor is actively moving", false);
    PortBuilder::createIntPort(returns, "speed_hz", "Speed (steps/s)",
                               "Current instantaneous speed in steps per second", false);

    def.returns = returns;

    agent->registerFunction(
        "get_stepper_state",
        def,
        [](Agent &agent, JsonObject args, JsonObject &returns) -> bool
        {
            if (!stepperInitialized || stepper == nullptr)
            {
                // Return safe zero-state when hardware is not present
                returns["current_position"] = 0;
                returns["target_position"] = 0;
                returns["is_running"] = false;
                returns["speed_hz"] = 0;
                return true;
            }

            int32_t currentPos = stepper->getCurrentPosition();
            int32_t targetPos = stepper->getPositionAfterCommandsCompleted();
            bool isRunning = stepper->isRunning();
            // getCurrentSpeedInMilliHz returns speed * 1000; convert to Hz
            int32_t speedHz = (int32_t)(stepper->getCurrentSpeedInMilliHz() / 1000);

            returns["current_position"] = currentPos;
            returns["target_position"] = targetPos;
            returns["is_running"] = isRunning;
            returns["speed_hz"] = speedHz;

            Serial.println("get_stepper_state  pos=" + String(currentPos) +
                           "  target=" + String(targetPos) +
                           "  running=" + String(isRunning ? "yes" : "no") +
                           "  speed=" + String(speedHz) + " Hz");
            return true;
        });
}

// ── Register all stepper functions ────────────────────────────────────────────
void registerAllStepperFunctions(Agent *agent)
{
    Serial.println("\n=== Registering Stepper Motor Functions ===");

    initStepper();
    registerStepperMove(agent);
    registerStepperStop(agent);
    registerStepperGetState(agent);

    Serial.println("✓ All stepper motor functions registered\n");
}

#endif // STEPPER_MOTOR_H
