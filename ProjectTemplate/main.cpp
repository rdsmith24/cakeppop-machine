// Program written by Roger Smith
// July 1, 2021
// For Ruf Engineering
// Cake Pop machine controls

// Includes
#include "ClearCore.h"

// ***********
// Definitions
#define LowerConnector ConnectorA9
#define ResetConnector ConnectorA10
#define myEStopConnector ConnectorA11
#define InputConnectorProx ConnectorDI6
#define InjectConnector ConnectorDI7
#define RaiseConnector ConnectorDI8
#define EndStopConnector ConnectorA12
#define baudRate 9600
#define SerialPort ConnectorUsb
#define motor ConnectorM0

// LCD Stuff
#define NUM_ROWS 4
#define NUM_COLUMNS 20
#define SPIbaudRate 80000
#define clockPolarity SerialDriver::SCK_HIGH
#define clockPhase SerialDriver::LEAD_CHANGE
#define SpiPort ConnectorCOM0


// *******************
// Function Prototypes
void SetupInterruptDI();
bool SetupSerialPort();
void DI_Callback();
void InjectCake();
void ReadAnalog();
void DisplayAnalog();
void SetupMotor();
void GetMotorTorque();
bool MoveDistance(int32_t distance);
bool ReadInputButtons();
void MaxTorqueDetected();
bool MoveAtVelocity(int32_t velocity);
void ConfigureDigitalInputs();
bool MoveToTorque();
bool HomeTheSystem();
void SetBrightness(uint8_t level);
void SetCursor(uint8_t row, uint8_t column);
void ConfigureSPIPort();
void writeLCDData();

// ****************
// Global Variables
bool processInjectCake = false;	
int  loops = 0;
bool serialPortOkay = false;	
bool bothButtonsDepressed = false;
bool manualMoveInProgress = false;
bool homeWasSucessful = false;
bool homingInProgress = false;
bool onlyInjectOnceFlag = false;
bool homeingRequestFlag = false;
int  machineState = 0;
bool eStopDetected = false;


int8_t torque = 0;
int8_t peaktorque = 0;
int8_t temptorque = 0;
int8_t temppeaktorque = 0;

// Variables to tune
int8_t peakTorqueMax = 95;
int32_t retractConstant = 19200;
int32_t RLVelocityConstant = 14000;  //Wolfgang change this to make raise/lower go faster
int32_t InjectionVelocityConstant = 25600; //Wolfgang change this to make injection go faster

// Define the velocity and acceleration limits to be used for each move
int32_t velocityLimit = 30000; // pulses per sec  //Change velocity limit on motor here
int32_t accelerationLimit = 30000; // pulses per sec^2

int32_t homingSpeed = 12800;  //Change homing speed here

// Sample data to write to the display
const uint8_t line1[21] = "  Ruf Engineering   ";
const uint8_t line2[21] = "CakePop Machine v1.0";

const char *ReadyStates[5]={"ServoState=>Disabled","ServoState=>Enabling","ServoState=--->Fault","ServoState=--->Ready","ServoState=-->Moving"};
const char *MachineStates[7]={"MachState=-->Startup","MachState=--->Homing","MachState=----->Idle","MachState=>Injecting","MachState=->ManRaise","MachState=->ManLower","MachState=--->E-Stop"};
	

// This section defines a periodic interrupt to write data to the LCD
//*******************************************************************
void ConfigurePeriodicInterrupt(uint32_t frequencyHz);
extern "C" void TCC2_0_Handler(void) __attribute__((
alias("PeriodicInterrupt")));
// Periodic interrupt priority
// 0 is highest priority, 7 is lowest priority
// Recommended priority is >= 4 to not interfere with other processing
#define PERIODIC_INTERRUPT_PRIORITY     4
#define ACK_PERIODIC_INTERRUPT  TCC2->INTFLAG.reg = TCC_INTFLAG_MASK
// State currently written to LED_BUILTIN
bool ledState = false;
uint32_t interruptFreqHz = 1;

extern "C" void PeriodicInterrupt(void) {
	// Perform periodic processing here.
	ledState = !ledState;
	ConnectorLed.State(ledState);
	writeLCDData();
	// Acknowledge the interrupt to clear the flag and wait for the next interrupt.
	ACK_PERIODIC_INTERRUPT;
}
//*******************************************************************


int main(void) {
	
	// Functions called one time only for setup of:
	// Serial Port
	// Interrupts
	// Analog Input ADC
	// Configure motor
	
	ConfigurePeriodicInterrupt(interruptFreqHz);
	//serialPortOkay = SetupSerialPort();
	SetupInterruptDI();
	ConfigureDigitalInputs();
	ConfigureSPIPort();
	SetupMotor();
	
	homeWasSucessful = HomeTheSystem();
	
	if(!homeWasSucessful) return(false); // We died....could not home
	
		
	// Main Loop starts here, program runs here till power is removed
	while (true) {
		
		machineState = 2;  //Signal that we are in idle mode
		// Check to see if the Emergency Stop is detected (open loop)
		while(!myEStopConnector.State()){
			eStopDetected = true;
			if(serialPortOkay) SerialPort.SendLine("Main...EStop Detected");
			machineState = 6;  //Signal that we are in idle mode
			continue;
		}
		
		// If we run this, we have just cleared an E-Stop.  Reset the motor and home the system
		if(eStopDetected){
			ConnectorM0.ClearAlerts();
			motor.EnableRequest(true);
			Delay_ms(200);
			eStopDetected = false;
			homeingRequestFlag = true;
		}
		
		// Watch to see if a cake injection has been requested?
		if(processInjectCake){
			machineState = 3;  //Signal that we are in injection mode
			InjectCake();
		}
		
		// Check to read if any operator input buttons have been pressed?
		ReadInputButtons();
		
		// The following structure is used to display various debug messages.
		if(loops > 30000){
			DisplayAnalog();
			loops = 0;
		} else {
			loops++;
		}
		
		// Check to see if homing was requested
		if(homeingRequestFlag){
			motor.EnableRequest(false);
			Delay_ms(200);
			motor.EnableRequest(true);
			Delay_ms(200);
			ConnectorM0.ClearAlerts();
			homeWasSucessful = HomeTheSystem();
			if(homeWasSucessful) homeingRequestFlag = false;
		}
	}// End Main Loop
}// End function main()


// *****************************************************************************************
// Function to setup an interrupt to initiate a "home" when the EndStopConnector is detected
void SetupInterruptDI(){
	// Configure the connector on the ClearCore defined by the variable "EndStopConnector" 
	// to detect interrupts
	EndStopConnector.Mode(Connector::INPUT_DIGITAL);
	EndStopConnector.InterruptHandlerSet(DI_Callback, InputManager::FALLING,false);
	EndStopConnector.InterruptEnable(true);
}

// *****************************************************************************************
// Function (callback) called when the digital interrupt is detected on the EndStopConnector
void DI_Callback() {
	// This is the function that is called when an interrupt on the connector "EndStopConnector" is detected
	homeingRequestFlag = true;
}

// **********************************************************************************
// Function to setup the serial port to write date back to the computer for debugging
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


// ************************************************
// Read the state of the buttons on the front panel
bool ReadInputButtons(){
	
	bool atVelocity = false;
	temptorque = 0;  // Clear the temporary torque variable for use in local logic
	
	if(homingInProgress) return(false);  // Do not read buttons while homing
	
	// Check to make sure that the handle is down and the injection button is pressed.
	// Button must have previously transitioned to the opposite state.
	if(InputConnectorProx.State() && !InjectConnector.State()) onlyInjectOnceFlag = true;
	if(!InputConnectorProx.State() && InjectConnector.State() && onlyInjectOnceFlag){
		processInjectCake = true;  // Signal that we need to inject cake
		onlyInjectOnceFlag = false;
	}
	
	// Check to see if operator is pressing the Raise Button?  Can't acknowledge request if homing is being requested.
	while(RaiseConnector.State() && !homeingRequestFlag){
		manualMoveInProgress = true;
		
		machineState = 4;  //Signal that we are in manual raise mode
		
		if(serialPortOkay) SerialPort.SendLine("ReadInputButtons...Motor Raise");
		// Check to see if we are at the requested velocity if not, request it.
		if(!atVelocity) atVelocity = MoveAtVelocity(-RLVelocityConstant);
		GetMotorTorque();
		if(torque > temptorque && torque < 100) temppeaktorque = torque;
		if(temppeaktorque > peakTorqueMax){
			MaxTorqueDetected();
			break;
		}
		continue;
	}
	// Stop manual move if at move was requested
	if(manualMoveInProgress){
		MoveAtVelocity(0);
		manualMoveInProgress = false;
	}

    // Check to see if the operator is pressing the Lower Button?  Can't acknowledge request if homing is being requested.
	while(LowerConnector.State() && !homeingRequestFlag){
		manualMoveInProgress = true;
		
		machineState = 5;  //Signal that we are in manual lower mode
		
		if(serialPortOkay) SerialPort.SendLine("ReadInputButtons...Motor Lower");
		// Check to see if we are at the requested velocity if not, request it.
		if(!atVelocity) atVelocity = MoveAtVelocity(RLVelocityConstant);
		GetMotorTorque();
		if(torque > temptorque && torque < 100) temppeaktorque = torque;
		if(temppeaktorque > peakTorqueMax){
			MaxTorqueDetected();
			break;
		}
		continue;
	}
	// Stop manual move if at move was requested
	if(manualMoveInProgress){
		MoveAtVelocity(0);
		manualMoveInProgress = false;
	}

    //  Check to see if the operator has pressed the Reset Button?  Can't acknowledge request if homing is being requested.
	if(ResetConnector.State() && !homeingRequestFlag){
		if(serialPortOkay) SerialPort.SendLine("ReadInputButtons...Home the motor initiated");
		motor.EnableRequest(false);
		Delay_ms(100);
		motor.EnableRequest(true);
		homeWasSucessful = HomeTheSystem();
		if(serialPortOkay) SerialPort.SendLine("ReadInputButtons...Home the motor complete");
	}
	return(true);
}

// *******************************************
// Function to initiate injection of the cake.
void InjectCake(){
	// If we have reached this point, we've been requested to inject cake batter
	if(serialPortOkay) SerialPort.SendLine("InjectCake...Injecting Cake");
	MoveToTorque();
	//Delay_ms(3000);
	// After injecting cake, move the lead screw back by the value in retractConstant.
	MoveDistance(-retractConstant);
	
	// Clear the inject cake flag
	processInjectCake = false;
}

void DisplayAnalog(){
	// Display various debug readings to the serial port.
	if(serialPortOkay){
		if(serialPortOkay) SerialPort.Send("Continuous Torque = ");
		if(serialPortOkay) SerialPort.Send(peaktorque * 3.861);  // Assume that 25.9% is maximum continuous torque of peak
		if(serialPortOkay) SerialPort.Send("%, ");
		if(serialPortOkay) SerialPort.Send("Peak Torque = ");
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
	
	// Set the motors positive limit switch input
	motor.LimitSwitchPos(CLEARCORE_PIN_A12);
	
	// Set the motors e-stop switch input
	motor.EStopConnector(CLEARCORE_PIN_A11);
	
	// Enables the motor; homing will begin automatically if enabled
	motor.EnableRequest(true);
	if(serialPortOkay) SerialPort.SendLine("SetupMotor...Motor Enabled");
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

void ConfigureDigitalInputs(){
	LowerConnector.Mode(Connector::INPUT_DIGITAL);
	ResetConnector.Mode(Connector::INPUT_DIGITAL);
	myEStopConnector.Mode(Connector::INPUT_DIGITAL);
	EndStopConnector.Mode(Connector::INPUT_DIGITAL);
	ConnectorIO5.Mode(Connector::OUTPUT_DIGITAL);
	ConnectorIO5.State(true);
}

// ***********************************************************************
// Function to move the motor, injecting cake until a shutdown is detected
bool MoveToTorque() {
	
	bool atVelocity = false;
	torque = 0;
	
	// Check if an alert is currently preventing motion
	if (motor.StatusReg().bit.AlertsPresent) {
		if(serialPortOkay) SerialPort.SendLine("MoveToTorque...Motor status: 'In Alert'. Move Canceled.");
		return false;
	}
	
	peaktorque = 0;

	// Move the motor to inject cake till the motor stops
	while(!motor.StatusReg().bit.AlertsPresent){
		if(!atVelocity) atVelocity = MoveAtVelocity(InjectionVelocityConstant);
		GetMotorTorque();
		peaktorque = torque;
		continue;
	}
	
	// Wait for a bit then re-enable the motor
	Delay_ms(300);
	motor.EnableRequest(false);
	ConnectorM0.ClearAlerts();
	Delay_ms(300);
	motor.EnableRequest(true);
	ConnectorM0.ClearAlerts();
	
	// Make sure to cancel the move when we get the alert
	MoveAtVelocity(0);
	if(serialPortOkay) SerialPort.SendLine("MoveToTorque...Move Done");
	return true;
}

// *********************************************************************************
// Function to home the system on startup, reset button and when end-stop is reached
bool HomeTheSystem(){
	
	homingInProgress = true;
	
	machineState = 1;  //Signal that we are in homing mode
	
	// Check if an alert would prevent motion
	if (motor.StatusReg().bit.AlertsPresent) {
		// In this case, we can't proceed with homing.
		if(serialPortOkay) SerialPort.SendLine("HomeTheSystem...Motor status: 'In Alert'. Move Canceled.");
		// The end...
		homingInProgress = false;
		return(false);
	}
	// Commands a speed of 5000 pulses/sec towards the hard stop for 2 seconds
	if(serialPortOkay) SerialPort.SendLine("HomeTheSystem...Moving toward hard stop... Waiting for HLFB");
	motor.MoveVelocity(round(-homingSpeed * 1.5));
	Delay_ms(2000);
	
	// Then slows down to 1000 pulses/sec until clamping into the hard stop
	motor.MoveVelocity(-homingSpeed);
	
	// Delay so HLFB has time to de-assert
	Delay_ms(10);
	
	// Waits for HLFB to assert again, meaning the hard stop has been reached
	while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
		continue;
	}
	// Stop the velocity move now that the hard stop is reached
	motor.MoveStopAbrupt();
	
	// Move away from the hard stop. Any move away from the hard stop will
	// conclude the homing sequence.
	motor.Move(3200);  
	
	// Delay so HLFB has time to de-assert
	Delay_ms(10);
	
	// Waits for HLFB to assert, meaning homing is complete
	if(serialPortOkay) SerialPort.SendLine("HomeTheSystem...Moving away from hard stop... Waiting for HLFB");
	while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
		continue;
	}
	if(serialPortOkay) SerialPort.SendLine("HomeTheSystem...Homing Complete. Motor Ready.");
	
	// Zero the motor's reference position after homing to allow for accurate
	// absolute position moves
	motor.PositionRefSet(0);
	homingInProgress = false;
	return(true);
}

// *************************************************
// Function to setup system for a periodic interrupt
void ConfigurePeriodicInterrupt(uint32_t frequencyHz) {
	// Enable the TCC2 peripheral.
	// TCC2 and TCC3 share their clock configuration and they
	// are already configured to be clocked at 120 MHz from GCLK0.
	CLOCK_ENABLE(APBCMASK, TCC2_);
	
	// Disable TCC2.
	TCC2->CTRLA.bit.ENABLE = 0;
	SYNCBUSY_WAIT(TCC2, TCC_SYNCBUSY_ENABLE);
	
	// Reset the TCC module so we know we are starting from a clean state.
	TCC2->CTRLA.bit.SWRST = 1;
	while (TCC2->CTRLA.bit.SWRST) {
		continue;
	}
	
	// If the frequency requested is zero, disable the interrupt and bail out.
	if (!frequencyHz) {
		NVIC_DisableIRQ(TCC2_0_IRQn);
		return;
	}
	
	// Determine the clock prescaler and period value needed to achieve the
	// requested frequency.
	uint32_t period = (CPU_CLK + frequencyHz / 2) / frequencyHz;
	uint8_t prescale;
	
	// Make sure period is >= 1.
	period = max(period, 1U);
	
	// Prescale values 0-4 map to prescale divisors of 1-16,
	// dividing by 2 each increment.
	for (prescale = TCC_CTRLA_PRESCALER_DIV1_Val;
	prescale < TCC_CTRLA_PRESCALER_DIV16_Val && (period - 1) > UINT16_MAX;
	prescale++) {
		period = period >> 1;
	}
	
	// Prescale values 5-7 map to prescale divisors of 64-1024,
	// dividing by 4 each increment.
	for (; prescale < TCC_CTRLA_PRESCALER_DIV1024_Val && (period - 1) > UINT16_MAX;
	prescale++) {
		period = period >> 2;
	}
	
	// If we have maxed out the prescaler and the period is still too big,
	// use the maximum period. This results in a ~1.788 Hz interrupt.
	if (period > UINT16_MAX) {
		TCC2->PER.reg = UINT16_MAX;
	}
	else {
		TCC2->PER.reg = period - 1;
	}
	TCC2->CTRLA.bit.PRESCALER = prescale;
	
	// Interrupt every period on counter overflow.
	TCC2->INTENSET.bit.OVF = 1;
	
	// Enable TCC2.
	TCC2->CTRLA.bit.ENABLE = 1;
	
	// Set the interrupt priority and enable it.
	NVIC_SetPriority(TCC2_0_IRQn, PERIODIC_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(TCC2_0_IRQn);
}

// **********************************
// Function to configure the SPI port
void ConfigureSPIPort(){
	
	SpiPort.Mode(Connector::SPI);
	SpiPort.Speed(SPIbaudRate);
	SpiPort.DataOrder(SerialDriver::COM_MSB_FIRST);
	SpiPort.SpiClock(clockPolarity, clockPhase);
	SpiPort.PortOpen();
	
	// The COM port is now configured and ready to send commands and
	// data to the display.
	// Set the display brightness level.
	// The maximum value for full brightness is 8.
	SetBrightness(4);
	
	// Set the cursor position to the top-left corner.
	SetCursor(0, 0);
}

// Fuction to set LCD brightness
void SetBrightness(uint8_t level) {
	SpiPort.SpiSsMode(SerialDriver::LINE_ON);
	SpiPort.SpiTransferData(0xfe);
	SpiPort.SpiTransferData(0x53);
	SpiPort.SpiTransferData(level);
	SpiPort.SpiSsMode(SerialDriver::LINE_OFF);
}

// Function to set cursor position on LCD
void SetCursor(uint8_t row, uint8_t column) {
	// Bounds-check the passed-in row and column
	if (row >= NUM_ROWS) {
		row = 0;
	}
	if (column >= NUM_COLUMNS) {
		column = 0;
	}
	uint8_t position = row * NUM_COLUMNS + column;
	SpiPort.SpiSsMode(SerialDriver::LINE_ON);
	SpiPort.SpiTransferData(0xfe);
	SpiPort.SpiTransferData(0x45);
	SpiPort.SpiTransferData(position);
	SpiPort.SpiSsMode(SerialDriver::LINE_OFF);
}

void writeLCDData(){
	
	// Set the cursor position to the top-left corner.
	SetCursor(0, 0);
	
	// Open the SPI port on ConnectorCOM0.
	SpiPort.SpiSsMode(SerialDriver::LINE_ON);
		
	// Send the lines "out of order" (1, 3, 2, 4) to the display.
	// Without resetting the cursor position for each line, this is the order
	// in which lines must be sent to be displayed correctly.
	SpiPort.SpiTransferData(line1, NULL, 20);
	SpiPort.SpiTransferData((uint8_t *)MachineStates[machineState], NULL, 20);
	SpiPort.SpiTransferData(line2, NULL, 20);
	SpiPort.SpiTransferData((uint8_t *)ReadyStates[motor.StatusReg().bit.ReadyState], NULL, 20);
	// Close the port.
	SpiPort.SpiSsMode(SerialDriver::LINE_OFF);
}