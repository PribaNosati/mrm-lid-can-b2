#include "mrm-lid-can-b2.h"
#include <mrm-robot.h>

std::map<int, std::string>* Mrm_lid_can_b2::commandNamesSpecific = NULL;

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_lid_can_b2::Mrm_lid_can_b2(uint8_t maxNumberOfBoards) : 
	SensorBoard(1, "Lid4m", maxNumberOfBoards, ID_MRM_LID_CAN_B2, 1) {
	readings = new std::vector<uint16_t>(maxNumberOfBoards);

	if (commandNamesSpecific == NULL){
		commandNamesSpecific = new std::map<int, std::string>();
		commandNamesSpecific->insert({COMMAND_LID_CAN_B2_DISTANCE_MODE, 	"Dist mode"});
		commandNamesSpecific->insert({COMMAND_LID_CAN_B2_TIMING_BUDGET, "Tim budge"});
		commandNamesSpecific->insert({COMMAND_LID_CAN_B2_MEASUREMENT_TIME, 	"Meas time"});
		commandNamesSpecific->insert({COMMAND_LID_CAN_B2_ROI, "ROI"});
	}
}

Mrm_lid_can_b2::~Mrm_lid_can_b2()
{
}

/** Add a mrm-lid-can-b device
@param deviceName - device's name
*/
void Mrm_lid_can_b2::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_LID_CAN_B2_0_IN;
		canOut = CAN_ID_LID_CAN_B2_0_OUT;
		break;
	case 1:
		canIn = CAN_ID_LID_CAN_B2_1_IN;
		canOut = CAN_ID_LID_CAN_B2_1_OUT;
		break;
	case 2:
		canIn = CAN_ID_LID_CAN_B2_2_IN;
		canOut = CAN_ID_LID_CAN_B2_2_OUT;
		break;
	case 3:
		canIn = CAN_ID_LID_CAN_B2_3_IN;
		canOut = CAN_ID_LID_CAN_B2_3_OUT;
		break;
	case 4:
		canIn = CAN_ID_LID_CAN_B2_4_IN;
		canOut = CAN_ID_LID_CAN_B2_4_OUT;
		break;
	case 5:
		canIn = CAN_ID_LID_CAN_B2_5_IN;
		canOut = CAN_ID_LID_CAN_B2_5_OUT;
		break;
	case 6:
		canIn = CAN_ID_LID_CAN_B2_6_IN;
		canOut = CAN_ID_LID_CAN_B2_6_OUT;
		break;
	case 7:
		canIn = CAN_ID_LID_CAN_B2_7_IN;
		canOut = CAN_ID_LID_CAN_B2_7_OUT;
		break;
	default:
		sprintf(errorMessage, "Too many %s: %i.", _boardsName.c_str(), nextFree);
		return;
	}
	SensorBoard::add(deviceName, canIn, canOut);
}

/** Calibration, only once after production
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
*/
void Mrm_lid_can_b2::calibration(Device * device){
	if (device == nullptr)
		for (Device& dev : devices)
			calibration(&dev);
	else{
		canData[0] = COMMAND_LID_CAN_B2_CALIBRATE;
		messageSend(canData, 1, device->number);
	}
}

std::string Mrm_lid_can_b2::commandName(uint8_t byte){
	auto it = commandNamesSpecific->find(byte);
	if (it == commandNamesSpecific->end())
		return "Warning: no command found for key " + (int)byte;
	else
		return it->second;//commandNamesSpecific->at(byte);
}

/** Reset sensor's non-volatile memory to defaults (distance mode, timing budget, region of interest, and measurement time, but leaves CAN Bus id intact
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0. 0xFF - resets all.
*/
void Mrm_lid_can_b2::defaults(uint8_t deviceNumber) {
	timingBudget(deviceNumber);
	delayMs(50); // Allow 50 ms for flash to be written
	measurementTime(deviceNumber);
	delayMs(50);
	distanceMode(deviceNumber);
	delayMs(50);
	roi(deviceNumber);
}

/** Distance in mm. Warning - the function will take considerable amount of time to execute if sampleCount > 0!
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param sampleCount - Number or readings. 40% of the readings, with extreme values, will be discarded and the
				rest will be averaged. Keeps returning 0 till all the sample is read.
				If sampleCount is 0, it will not wait but will just return the last value.
@param sigmaCount - Values outiside sigmaCount sigmas will be filtered out. 1 sigma will leave 68% of the values, 2 sigma 95%, 3 sigma 99.7%.
				Therefore, lower sigma number will remove more errornous readings.
@return - distance in mm
*/
uint16_t Mrm_lid_can_b2::distance(uint8_t deviceNumber, uint8_t sampleCount, uint8_t sigmaCount){
	const uint16_t TIMEOUT = 3000;
	if (deviceNumber >= nextFree) {
		strcpy(errorMessage, "mrm-lid-can-b2 doesn't exist");
		return 0;
	}
	if (started(devices[deviceNumber]))
		if (sampleCount == 0)
			return (*readings)[deviceNumber] == 0 ? 4000 : (*readings)[deviceNumber];
		else{
			uint16_t rds[sampleCount];
			for (uint8_t i = 0; i < sampleCount; i++){
				if (i != 0) // For 2. reading, etc. - force new readout
					(*readings)[deviceNumber] = 0;
				uint32_t ms = millis();
				while ((*readings)[deviceNumber] == 0){
					noLoopWithoutThis();
					if (millis() - ms > TIMEOUT){
						errorAdd(CANMessage(devices[deviceNumber].canIdIn, {0}, 0), ERROR_TIMEOUT, false, false);
						break;
					}
				}
				rds[i] = (*readings)[deviceNumber];
				//print("Reading %i\n\r", (*readings)[deviceNumber]);
			}

			// Average and standard deviation
			float mean;
			float sd = stardardDeviation(sampleCount, rds, &mean);

			return outlierlessAverage(sampleCount, rds, mean, sigmaCount, sd);
		}
	else
		return 0;
}

/** Distance mode. Short mode has better ambient light immunity but the maximum distance is limited to 1.3 m. Long distance ranges up to
	4 m but is less performant under ambient light. Stored in sensors non-volatile memory. Allow 50 ms for flash to be written.
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param isShort - if not short, long.
*/
void Mrm_lid_can_b2::distanceMode(uint8_t deviceNumber, bool isShort) {
	if (deviceNumber == 0xFF) {
		for (uint8_t i = 0; i < nextFree; i++)
			if (aliveWithOptionalScan(&devices[i])) {
				distanceMode(i, isShort);
				delay(1);
			}
	}
	else {
		canData[0] = COMMAND_LID_CAN_B2_DISTANCE_MODE;
		canData[1] = isShort;
		messageSend(canData, 2, deviceNumber);
	}
}


/** Measurement time (IMP) in ms. IMP must be >= TB. Probably the best value is IMP = TB. Stored in sensors non-volatile memory.
Allow 50 ms for flash to be written.
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param ms - default 100.
*/
void Mrm_lid_can_b2::measurementTime(uint8_t deviceNumber, uint16_t ms) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			measurementTime(i, ms);
	else if (aliveWithOptionalScan(&devices[deviceNumber])) {
		canData[0] = COMMAND_LID_CAN_B2_MEASUREMENT_TIME;
		canData[1] = ms & 0xFF;
		canData[2] = ms >> 8;
		messageSend(canData, 3, deviceNumber);
	}
}


/** Read CAN Bus message into local variables
@param canId - CAN Bus id
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
@return - true if canId for this class
*/
bool Mrm_lid_can_b2::messageDecode(CANMessage& message) {
	for (Device& device : devices)
		if (isForMe(message.id, device)) {
			if (!messageDecodeCommon(message, device)) {
				switch (message.data[0]) {
				case COMMAND_SENSORS_MEASURE_SENDING: {
					uint16_t mm = (message.data[2] << 8) | message.data[1];
					(*readings)[device.number] = mm;
					device.lastReadingsMs = millis();
				}
				break;
				case COMMAND_INFO_SENDING_1:
					print("%s: %s dist., budget %i ms, %ix%i, intermeas. %i ms\n\r", device.name.c_str(), message.data[1] ? "short" : "long", message.data[2] | (message.data[3] << 8),
						message.data[4] & 0xFF, message.data[5] & 0xFF, message.data[6] | (message.data[7] << 8));
					break;
				default:
					errorAdd(message, ERROR_COMMAND_UNKNOWN, false, true);
				}
			}
			return true;
		}
	return false;
}


/** Analog readings
@param receiverNumberInSensor - always 0
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_lid_can_b2::reading(uint8_t receiverNumberInSensor, uint8_t deviceNumber){
	return distance(deviceNumber);
}

/** Print all readings in a line
*/
void Mrm_lid_can_b2::readingsPrint() {
	print("Lid4m:");
	for (Device& device: devices)
		if (device.alive)
			print(" %4i", distance(device.number));
}

/** ROI, region of interest, a matrix from 4x4 up to 16x16 (x, y). Smaller x and y - smaller view angle. Stored in sensors non-volatile memory.
Allow 50 ms for flash to be written.
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param x - default 16.
@param y - default 16.
*/
void Mrm_lid_can_b2::roi(uint8_t deviceNumber, uint8_t x, uint8_t y) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			roi(i, x, y);
	else if (aliveWithOptionalScan(&devices[deviceNumber])) {
		delay(1);
		canData[0] = COMMAND_LID_CAN_B2_ROI;
		canData[1] = x;
		canData[2] = y;
		messageSend(canData, 3, deviceNumber);
	}
}


/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_lid_can_b2::started(Device& device) {
	if (millis() - device.lastReadingsMs > MRM_LID_CAN_B2_INACTIVITY_ALLOWED_MS || device.lastReadingsMs == 0) {
		//print("Start mrm-lid-can-b2%i \n\r", deviceNumber);
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(&device, 0);
			// Wait for 1. message.
			uint64_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - device.lastReadingsMs < 100) {
					//print("Lidar confirmed\n\r");
					return true;
				}
				delay(1);
			}
		}
		sprintf(errorMessage, "%s %i dead.", _boardsName.c_str(), device.number);
		return false;
	}
	else
		return true;
}


/**Test
*/
void Mrm_lid_can_b2::test(uint16_t betweenTestsMs)
{
	static uint64_t lastMs = 0;
	if (millis() - lastMs > (betweenTestsMs == 0 ? 300 : betweenTestsMs)) {
		uint8_t pass = 0;
		for (Device& device: devices){
			if (device.alive) {
				if (pass++)
					print(" ");
				print("%i ", distance(device.number));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}


/** Timing budget (TB) in ms. TB improves the measurement reliability but increases power consumption. Stored in sensors non-volatile memory.
	Set before measurement time as measurement time checks this value and returns error if not appropriate. Allow 50 ms for flash to be written.
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@param ms - (TB) in ms possible values [20, 50, 100, 200, 500]. Default 100.
*/
void Mrm_lid_can_b2::timingBudget(uint8_t deviceNumber, uint16_t ms) {
	if (deviceNumber == 0xFF)
		for (uint8_t i = 0; i < nextFree; i++)
			timingBudget(i, ms);
	else if (aliveWithOptionalScan(&devices[deviceNumber])) {
		delay(1);
		canData[0] = COMMAND_LID_CAN_B2_TIMING_BUDGET;
		canData[1] = ms & 0xFF;
		canData[2] = ms >> 8;
		messageSend(canData, 3, deviceNumber);
	}
}

