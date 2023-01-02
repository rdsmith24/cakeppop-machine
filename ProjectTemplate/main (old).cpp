// Program written by Roger Smith and Justin "Snake Pliskin" Smith
// July 1, 2021
// For Ruf Engineering
// Cake Pop machine controls

// Includes
#include "ClearCore.h"

// Definitions
#define LowerConnector ConnectorA9
#define ResetConnector ConnectorA10
#define EStopConnector ConnectorA11
#define InputConnectorProx ConnectorDI6
#define InputConnectorButton ConnectorDI7
#define RaiseConnector ConnectorDI8
#define baudRate 9600
#define SerialPort ConnectorUsb
#define adcResolution 12
#define motor ConnectorM0

// Function Prototypes
void SetupInterruptDI();
void SetupAI();
bool SetupSerialPort();
void DI_Callback();
void InjectCake();
void ReadAnalog();
void DisplayAnalog();
void SetupMotor();
void GetMotorTorque();
bool MoveDistance(int32_t distance);
void ReadInputButtons();
bool ButtonCheck();
void MaxTorqueDetected();
bool MoveAtVelocity(int32_t velocity);

// Global Variables
bool processInjectCake = false;	// System has signaled to inject cake batter
double inputVoltage;			// variable to read raw value from ADC
double bias = 0.0;				//  variable to use to bias cake batter pulse
int loops = 0;
bool serialPortOkay = false;	// Okay to send stuff up the serial port
bool bothButtonsDepressed = false;
bool manualMoveInProgress = false;
int8_t torque = 0;
int8_t peaktorque = 0;
int8_t temptorque = 0;
int8_t temppeaktorque = 0;

// Variable to tune
int8_t peakTorqueMax = 95;
int32_t injectionConstant = 25804;
int32_t retractConstant = 1600;
int32_t RLVelocityConstant = 3000;

// Define the velocity and acceleration limits to be used for each move
int32_t velocityLimit = 10000; // pulses per sec
int32_t accelerationLimit = 10000; // pulses per sec^2



int main(void) {
	
	// Functions called one time only for setup of:
	// Serial Port
	// Interrupts
	// Analog Input ADC
	// Configure motor
	
	serialPortOkay = SetupSerialPort();
	//SetupInterruptDI();
	SetupAI();
	SetupMotor();
	LowerConnector.Mode(Connector::INPUT_DIGITAL);
	ResetConnector.Mode(Connector::INPUT_DIGITAL);
	EStopConnector.Mode(Connector::INPUT_DIGITAL);
		
	// Main Loop starts here, program runs here till power is removed
	while (true) {
		
		// Check to see if the Emergency Stop is detected (open loop)
		while(!EStopConnector.State()){
			if(serialPortOkay) SerialPort.SendLine("Main...EStop Detected");
			continue;
		}
		
		// Watch to see if a cake injection has been requested?
		if(processInjectCake){
			InjectCake();
		}
		
		// Read the potentiometer and process the bias value
		ReadAnalog();
		
		// Check to read if any operator input buttons have been pressed?
		ReadInputButtons();
		
		// The following structure is used to display various debug messages.
		if(loops > 30000){
			DisplayAnalog();
			loops = 0;
		} else {
			loops++;
		}
	}// End Main Loop
}// End function main()



/*void SetupInterruptDI(){
	// Configure the connector on the ClearCore defined by the variable "interruptConnector" 
	// to detect interrupts
	interruptConnector.Mode(Connector::INPUT_DIGITAL);
	interruptConnector.InterruptHandlerSet(DI_Callback, InputManager::FALLING,false);
	interruptConnector.InterruptEnable(true);
}*/

void SetupAI(){
	// Set the resolution of the ADC.
	AdcMgr.AdcResolution(adcResolution);
}

bool SetupSerialPort(){
	// Set up serial communication and wait up to 5 seconds for a port to open
	// Serial communication is not required for this example to run.
	SerialPort.Mode(Connector::USB_CDC);
	SerialPort.Speed(baudRate);
	uint32_t timeout = 5000;
	uint32_t startTime = Milliseconds();
	SerialPort.PortOpen();
	while (!SerialPort && Milliseconds() - startTime < timeout) {
		continue;
	}
	return SerialPort.PortIsOpen() ? true : false;
}

/*void DI_Callback() {
	// This is the function that is called when an interrupt on the connector "interruptConnector" is detected
	ConnectorLed.State(true);
	Delay_ms(10);
	ConnectorLed.State(false);
	Delay_ms(10);
	processInjectCake = true;  // Signal that we need to inject cake
}*/

void ReadInputButtons(){
	temptorque = 0;  // Clear the temporary torque variable for use in local logic
	
	// Check to make sure that the handle is down and the button is pressed.
	// Both must have previously transitioned to the opposite state. (no taping of buttons)
	if(!InputConnectorProx.InputFallen() && InputConnectorButton.InputRisen()){
		ConnectorLed.State(true);
		Delay_ms(10);
		ConnectorLed.State(false);
		Delay_ms(10);
		processInjectCake = true;  // Signal that we need to inject cake
	}
	
	// Check to see if operator is pressing the Raise Button?
	while(RaiseConnector.State()){
		manualMoveInProgress = true;
		if(serialPortOkay) SerialPort.SendLine("ReadInputButtons...Motor Raise");
		MoveAtVelocity(-RLVelocityConstant);
		GetMotorTorque();
		if(torque > temptorque && torque < 100) temppeaktorque = torque;
		if(temppeaktorque > peakTorqueMax){
			MaxTorqueDetected();
			break;
		}
		continue;
	}
	// Stop manual move if detected
	if(manualMoveInProgress){
		MoveAtVelocity(0);
		manualMoveInProgress = false;
	}

    // Check to see if the operator is pressing the Lower Button?
	while(LowerConnector.State()){
		manualMoveInProgress = true;
		if(serialPortOkay) SerialPort.SendLine("ReadInputButtons...Motor Lower");
		MoveAtVelocity(RLVelocityConstant);
		GetMotorTorque();
		if(torque > temptorque && torque < 100) temppeaktorque = torque;
		if(temppeaktorque > peakTorqueMax){
			MaxTorqueDetected();
			break;
		}
		continue;
	}
	// Stop manual move if detected
	if(manualMoveInProgress){
		MoveAtVelocity(0);
		manualMoveInProgress = false;
	}

    //  Check to see if the operator has pressed the Reset Button?
	if(ResetConnector.State()){
		if(serialPortOkay) SerialPort.SendLine("ReadInputButtons...Motor Reset");
		motor.EnableRequest(false);
		Delay_ms(100);
			
		// Enables the motor; homing will begin automatically if enabled
		motor.EnableRequest(true);
		if(serialPortOkay) SerialPort.SendLine("ReadInputButtons...Motor Enabled");
	}
}

void InjectCake(){
	// Inject cake batter
	if(serialPortOkay) SerialPort.SendLine("InjectCake...Injecting Cake");
	MoveDistance(injectionConstant + retractConstant);
	MoveDistance(-retractConstant);
	processInjectCake = false;
}

void ReadAnalog(){
	// Convert the reading to a voltage.
	int16_t adcResult = ConnectorA12.State();
	inputVoltage = 10.0 * adcResult / ((1 << adcResolution) - 1);
	bias = inputVoltage * .04 + .8;  // Scale 0-10V to 0.8 to 1.2
}

void DisplayAnalog(){
	// Display various debug readings to the serial port.
	if(serialPortOkay){
		if(serialPortOkay) SerialPort.Send("DisplayAnalog...A-12 input voltage: ");
		if(serialPortOkay) SerialPort.Send(inputVoltage);
		if(serialPortOkay) SerialPort.Send("V. ");
		if(serialPortOkay) SerialPort.Send("Bias = ");
		if(serialPortOkay) SerialPort.Send(bias);
		if(serialPortOkay) SerialPort.Send("%");
		if(serialPortOkay) SerialPort.Send(" Peak Torque = ");
		if(serialPortOkay) SerialPort.Send(peaktorque);
		if(serialPortOkay) SerialPort.SendLine("%");
	}
}

void SetupMotor(){
	// Sets the input clocking rate. This normal rate is ideal for ClearPath
	// step and direction applications.
	MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

	// Sets all motor connectors into step and direction mode.
	MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

	// Set the motor's HLFB mode to bipolar PWM
	motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
	// Set the HFLB carrier frequency to 482 Hz
	motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

	// Sets the maximum velocity for each move
	motor.VelMax(velocityLimit);

	// Set the maximum acceleration for each move
	motor.AccelMax(accelerationLimit);
	
	// Enables the motor; homing will begin automatically if enabled
	motor.EnableRequest(true);
	if(serialPortOkay) SerialPort.SendLine("SetupMotor...Motor Enabled");
	
	// Waits for HLFB to assert (waits for homing to complete if applicable)
	if(serialPortOkay) SerialPort.SendLine("SetupMotor...Waiting for HLFB...");
	while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
		continue;
	}
	if(serialPortOkay) SerialPort.SendLine("SetupMotor...Motor Ready");
}

void GetMotorTorque(){
	// Reads motor state and percent torque while the motor is in motion
	// Updates global variable "torque"
	
	if(serialPortOkay) SerialPort.Send("GetMotorTorque...HLFB state: ");

	// Check the current state of the ClearPath's HLFB.
	MotorDriver::HlfbStates hlfbState = motor.HlfbState();

	// Write the HLFB state to the serial port
	if (hlfbState == MotorDriver::HLFB_HAS_MEASUREMENT) {
		torque = (abs(int8_t(round(motor.HlfbPercent()))));
		// Writes the torque measured, as a percent of motor peak torque rating
		if(serialPortOkay) SerialPort.Send(torque);
		if(serialPortOkay) SerialPort.SendLine("% torque");
	}
	else if (hlfbState == MotorDriver::HLFB_ASSERTED) {
		// Asserted indicates either "Move Done" for position modes, or
		// "At Target Velocity" for velocity moves
		if(serialPortOkay) SerialPort.SendLine("ASSERTED");
	}
	else {
		if(serialPortOkay) SerialPort.SendLine("DISABLED or SHUTDOWN");
	}
}

bool MoveDistance(int32_t distance) {
	// Command "distance" number of step pulses away from the current position
	// Prints the move status to the USB serial port
	// Returns when HLFB asserts (indicating the motor has reached the commanded
	// position)
	//
	// Parameters:
	// int distance  - The distance, in step pulses, to move
	//
	// Returns: True/False depending on whether the move was successfully triggered.
	
    // Check if an alert is currently preventing motion
    if (motor.StatusReg().bit.AlertsPresent) {
        if(serialPortOkay) SerialPort.SendLine("MoveDistance...Motor status: 'In Alert'. Move Canceled.");
        return false;
    }

    if(serialPortOkay) SerialPort.Send("MoveDistance...Moving distance: ");
    if(serialPortOkay) SerialPort.SendLine(distance);
	
	peaktorque = 0;

    // Command the move of incremental distance
    motor.Move(distance);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    if(serialPortOkay) SerialPort.SendLine("MoveDistance...Moving.. Waiting for HLFB");
    while (!motor.StepsComplete() || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
		GetMotorTorque();
		if(torque > peaktorque && torque < 100) peaktorque = torque;
		if(peaktorque > peakTorqueMax) MaxTorqueDetected();
        continue;
    }

    if(serialPortOkay) SerialPort.SendLine("MoveDistance...Move Done");
    return true;
}

void MaxTorqueDetected(){
	// Disables the motor and wait 100ms
	if(serialPortOkay) SerialPort.SendLine("MaxTorqueDetected...Motor Disabled");
	motor.EnableRequest(false);
	Delay_ms(100);
	
	// Enables the motor; homing will begin automatically if enabled
	motor.EnableRequest(true);
	if(serialPortOkay) SerialPort.SendLine("MaxTorqueDetected...Motor Enabled");
	
	// Waits for HLFB to assert (waits for homing to complete if applicable)
	if(serialPortOkay) SerialPort.SendLine("MaxTorqueDetected...Waiting for HLFB...");
	while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
		continue;
	}
	if(serialPortOkay) SerialPort.SendLine("MaxTorqueDetected...Motor Ready");
}

bool MoveAtVelocity(int32_t velocity) {
	// Check if an alert is currently preventing motion
	if (motor.StatusReg().bit.AlertsPresent) {
		if(serialPortOkay) SerialPort.SendLine("MoveAtVelocity...Motor status: 'In Alert'. Move Canceled.");
		return false;
	}

	if(serialPortOkay) SerialPort.Send("MoveAtVelocity...Commanding velocity: ");
	if(serialPortOkay) SerialPort.SendLine(velocity);

	// Command the velocity move
	motor.MoveVelocity(velocity);

	// Waits for the step command to ramp up/down to the commanded velocity.
	// This time will depend on your Acceleration Limit.
	if(serialPortOkay) SerialPort.SendLine("MoveAtVelocity...Ramping to speed...");
	while (!motor.StatusReg().bit.AtTargetVelocity) {
		continue;
	}

	if(serialPortOkay) SerialPort.SendLine("MoveAtVelocity...At Speed");
	return true;
}