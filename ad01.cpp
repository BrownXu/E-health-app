#include <Adafruit_GFX_AS.h>
#include <Adafruit_ILI9341_AS.h>
#include <MySignals.h>
#include <MySignals_BLE.h>
#include <Wire.h>
#include <SPI.h> 

//Global variables
char buffer_tft[30]; //TFT display
Adafruit_ILI9341_AS tft = Adafruit_ILI9341_AS(TFT_CS, TFT_DC);
unsigned long previous;
uint8_t valuetemperature[2]; //temperature
uint8_t pulsioximeter_state = 1; //state of scensor
uint8_t valueSPO2[4];
int16_t countern = 0;
uint8_t valueMEG[50]; // get 25 scans at one time use 50 byte 1 byte save lable, as BLE limit 54byte transfer
int megcount = 0; // count number of meg value
bool teststart; //start test lable
bool aleart; //get aleart
bool selected_emg;
bool selected_temp;
bool selected_spo2_uart;


void setup()
{
	Serial.begin(115200);
	MySignals.begin();
//	valuetemperature[0] = 0b00000001; //set temperature lable equal 1
//	valueSPO2[0] = 0b00000010; // set SPO2 lable equal 2
//	valueMEG[0] = 0b00001000; //set MEG lable equal 8
	MySignals.begin();
	tft.init();
	tft.setRotation(2);
	tft.fillScreen(ILI9341_BLACK);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

	//TFT message: Welcome 
	tft.drawString("Muscle Fatigue Monitor ", 0, 0, 2);

	MySignals.initSensorUART();

	MySignals.enableSensorUART(BLE);

	//Enable BLE module power -> bit6: 1
	bitSet(MySignals.expanderState, EXP_BLE_POWER);
	MySignals.expanderWrite(MySignals.expanderState);

	//Enable BLE UART flow control -> bit5: 0
	bitClear(MySignals.expanderState, EXP_BLE_FLOW_CONTROL);
	MySignals.expanderWrite(MySignals.expanderState);


	//Disable BLE module power -> bit6: 0
	bitClear(MySignals.expanderState, EXP_BLE_POWER);
	MySignals.expanderWrite(MySignals.expanderState);

	delay(500);

	//Enable BLE module power -> bit6: 1
	bitSet(MySignals.expanderState, EXP_BLE_POWER);
	MySignals.expanderWrite(MySignals.expanderState);
	delay(1000);

	MySignals_BLE.initialize_BLE_values();

	//Clean the input serial buffer
	while (Serial.available() > 0)
	{
		Serial.read();
	}


	if (MySignals_BLE.initModule() == 1)
	{

		if (MySignals_BLE.sayHello() == 1)
		{
			//TFT message: "BLE init ok";
			strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[1])));
			tft.drawString(buffer_tft, 0, 15, 2);
		}
		else
		{
			//TFT message:"BLE init fail"
			strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[2])));
			tft.drawString(buffer_tft, 0, 15, 2);


			while (1)
			{
			};
		}
	}
	else
	{
		//TFT message: "BLE init fail"
		strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[2])));
		tft.drawString(buffer_tft, 0, 15, 2);

		while (1)
		{
		};
	}

}
void loop()
{
	uint16_t rawemg;
	uint16_t rawtemperature;
	uint16_t rawPulse;
	uint16_t rawSPO2;
	uint8_t rawemg_low;
	uint8_t rawemg_high;
	uint8_t rawtemp_low;
	uint8_t rawtemp_high;
	uint8_t rawSPO2_low;
	uint8_t rawSPO2_high;
	uint8_t rawPulse_low;
	uint8_t rawPulse_high;

	teststart = 1;
	//SET BLE MODE
	while ((MySignals_BLE.ble_mode_flag == master_mode))
	{

		if (MySignals_BLE.setMode(slave_mode) == 0)
		{
			//TFT message:  "Slave mode ok";
			strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[3])));
			tft.drawString(buffer_tft, 0, 30, 2);
			MySignals_BLE.ble_mode_flag = slave_mode;
		}
		else
		{
			//TFT message:  "Slave mode fail"
			strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[4])));
			tft.drawString(buffer_tft, 0, 30, 2);
			MySignals_BLE.hardwareReset();
			delay(100);
			MySignals_BLE.initialize_BLE_values();
		}
	}
	// SET BONDABLE MODE
	if (MySignals_BLE.bond_mode_and_mitm == 0)
	{
		if (MySignals_BLE.setBondableMode(BLE_ENABLE_BONDING) == 0)
		{

			//TFT message:  "Bondable mode ok"
			strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[5])));
			tft.drawString(buffer_tft, 0, 45, 2);
			//3. SET SM PARAMETERS
			if (MySignals_BLE.setSMParameters(BLE_ENABLE_MITM) == 0)
			{
				//TFT message:  "SM parameters ok"
				strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[7])));
				tft.drawString(buffer_tft, 0, 60, 2);
				MySignals_BLE.bond_mode_and_mitm = 1;
			}
			else
			{
				//TFT message:  "SM parameters fail"
				strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[8])));
				tft.drawString(buffer_tft, 0, 60, 2);

				MySignals_BLE.hardwareReset();
				delay(100);
				MySignals_BLE.initialize_BLE_values();
			}
		}
		else
		{
			//TFT message:  "Bondable mode fail"
			strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[6])));
			tft.drawString(buffer_tft, 0, 45, 2);

			MySignals_BLE.hardwareReset();
			delay(100);
			MySignals_BLE.initialize_BLE_values();
		}
	}
	// BONDING AND CONNECTION CONFIGURATION
	if ((MySignals_BLE.ble_mode_flag == slave_mode) && (MySignals_BLE.bonded_and_connected_flag == 0))
	{
		Serial.println("slave or bonded");
		MySignals_BLE.bonding_correct = 0;
		MySignals_BLE.app_connected_flag = 0;
		MySignals_BLE.bonding_fail = 0;
		Serial.println("set flag123");

		/////////////////////

		//TFT message:  "Waiting connections..."
		strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[9])));
		tft.drawString(buffer_tft, 0, 75, 2);


		uint8_t flag = MySignals_BLE.waitEvent(500);
		Serial.println("event 500");

		if (flag == BLE_EVENT_CONNECTION_STATUS)
		{
			MySignals_BLE.app_connected_flag = 1;
			Serial.println("BLE_EVENT_CONNECTION_STATUS01");
		}
		else if (flag == BLE_EVENT_SM_BOND_STATUS)
		{
			if (MySignals_BLE.event[6] == 0x01)
			{
				MySignals_BLE.bonding_correct = 1;
				Serial.println("event[6] 0x01");
				delay(1000);
			}
		}
		else if (flag == 0)
		{
			// If there are no events, then no one tried to connect
		}
		else if (flag == BLE_EVENT_ATTRIBUTES_VALUE)
		{
			//Already connected
			MySignals_BLE.app_connected_flag = 1;
			MySignals_BLE.bonding_correct = 1;
			MySignals_BLE.bonded_and_connected_flag = 1;
			Serial.println("BLE_EVENT_ATTRIBUTES_VALUE001");
		}
		else
		{
			// Other event received from BLE module
		}

		/////////////////////

		if ((MySignals_BLE.bonding_correct == 1) || MySignals_BLE.app_connected_flag == 1)
		{
			//TFT message:  "Connection detected..."
			strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[10])));
			tft.drawString(buffer_tft, 0, 90, 2);
			Serial.print("bonding_correct");
			Serial.println(MySignals_BLE.bonding_correct);
			Serial.print("appflag");
			Serial.println(MySignals_BLE.app_connected_flag);
			previous = millis();

			while ((MySignals_BLE.bonded_and_connected_flag == 0) && (MySignals_BLE.bonding_fail == 0))
			{
				//  Timeout 30 sg
				if ((millis() - previous) > 30000)
				{
					MySignals_BLE.bonding_fail = 1;
					Serial.println("time out");
				}

				flag = MySignals_BLE.waitEvent(1000);

				if (flag == 0)
				{
					//Do nothing
				}
				else if (flag == BLE_EVENT_SM_PASSKEY_DISPLAY)
				{
					Serial.println("show pass");
					uint32_t passkey_temp =
						uint32_t(MySignals_BLE.event[5]) +
						uint32_t(MySignals_BLE.event[6]) * 256 +
						uint32_t(MySignals_BLE.event[7]) * 65536 +
						uint32_t(MySignals_BLE.event[8]) * 16777216;

					//TFT message:  "Passkey:";"
					strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[11])));
					tft.drawString(buffer_tft, 0, 105, 2);
					tft.drawNumber(passkey_temp, 50, 105, 2);
				}

				else if (flag == BLE_EVENT_ATTRIBUTES_VALUE)
				{
					//Already connected
					MySignals_BLE.app_connected_flag = 1;
					MySignals_BLE.bonding_correct = 1;
					MySignals_BLE.bonded_and_connected_flag = 1;
					Serial.println("BLE_EVENT_ATTRIBUTES_VALUE002");
				}

				else if (flag == BLE_EVENT_SM_BOND_STATUS)
				{

					if (MySignals_BLE.event[6] == 0x01)
					{
						//Man-in-the-Middle mode correct
						MySignals_BLE.bonding_correct = 1;
						Serial.println("event[6] 0x01002");
					}
				}

				else if (flag == BLE_EVENT_CONNECTION_FEATURE_IND)
				{
					//Do nothing
				}

				else if (flag == BLE_EVENT_CONNECTION_VERSION_IND)
				{
					//Do nothing
				}

				else if (flag == BLE_EVENT_SM_BONDING_FAIL)
				{
					MySignals_BLE.bonded_and_connected_flag = 0;
					MySignals_BLE.bonding_fail = 1;
					Serial.println("BLE_EVENT_SM_BONDING_FAIL");
				}
				else if (flag == BLE_EVENT_CONNECTION_STATUS)
				{

					if (MySignals_BLE.event[5] == 0x03)
					{
						//Connection correct
						MySignals_BLE.app_connected_flag = 1;
						Serial.println("event[5] 0x03");
					}
				}
				else if (flag == BLE_EVENT_CONNECTION_DISCONNECTED)
				{
					MySignals_BLE.bonded_and_connected_flag = 0;
					MySignals_BLE.bonding_fail = 1;
					Serial.println("BLE_EVENT_CONNECTION_DISCONNECTED");
				}
				else
				{
					//Do nothing
				}


				if (MySignals_BLE.bonding_correct && MySignals_BLE.app_connected_flag)
				{

					//TFT message:  "Connected!"
					strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[12])));
					tft.drawString(buffer_tft, 0, 120, 2);
					Serial.println("Connected");

					//TFT message:  "Sensor list:"
					strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[14])));
					tft.drawString(buffer_tft, 0, 135, 2);

					//// SENSORES

					//TFT message:  "Temperature:"
					strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[21])));
					tft.drawString(buffer_tft, 0, 150, 2);

					//TFT message:  "EMG:"
					strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[25])));
					tft.drawString(buffer_tft, 0, 165, 2);

					//TFT message:  "SPO2:"
					strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[28])));
					tft.drawString(buffer_tft, 0, 180, 2);

					MySignals_BLE.bonded_and_connected_flag = 1;
				}

			}

			if (MySignals_BLE.bonding_fail == 1)
			{
				//TFT message:  "Connection failed. Reseting"
				strcpy_P((char*)buffer_tft, (char*)pgm_read_word(&(table_MISC[13])));
				tft.drawString(buffer_tft, 0, 120, 2);
				Serial.println("bonding_fail");
				MySignals_BLE.bonded_and_connected_flag = 1;
				MySignals_BLE.hardwareReset();
				delay(100);
				MySignals_BLE.initialize_BLE_values();
			}
		}
	}

	// Start transfer data
	if ((MySignals_BLE.ble_mode_flag == slave_mode) && (MySignals_BLE.app_connected_flag == 1))
	{
//		SPI.end();
		MySignals.enableSensorUART(BLE);
//		uint8_t abc[3]= {0b10111110, 0b10010111,0b00000000};
//		MySignals_BLE.writeLocalAttribute(64,abc, 50);
		if (MySignals_BLE.readLocalAttribute(64) == 0)
		{
//			sprintf(buffer_tft, "%X %X %X  ", MySignals_BLE.attributeValue[0], MySignals_BLE.attributeValue[1], MySignals_BLE.attributeValue[2]);
//			tft.drawString(buffer_tft, 100, 135, 2);
			selected_emg = MySignals_BLE.attributeValue[1] & 0b00000010;
			selected_temp = MySignals_BLE.attributeValue[1] & 0b10000000;
			selected_spo2_uart = MySignals_BLE.attributeValue[1] & 0b00100000;
//			Serial.print("handle30");
//			Serial.println(MySignals_BLE.readLocalAttribute(64));
//			Serial.print("handle30value");
//			Serial.println(MySignals_BLE.attributeValue[0]);
//			Serial.println(MySignals_BLE.attributeValue[1]);
//			Serial.println(MySignals_BLE.attributeValue[2]);
		}
		if (teststart == 1)
		{
			if (megcount < 25)
			{
				rawemg = MySignals.getCalibratedEMG(3, 0, 0, DATA);
				rawemg_low = rawemg & 0b0000000011111111;
				rawemg_high = (rawemg & 0b1111111100000000) / 256;
				valueMEG[(megcount*2 )] = rawemg_high;
				valueMEG[(megcount*2+1)] = rawemg_low;
				megcount++;
//				Serial.print("emg");
//				Serial.println(rawemg);
			}
			else if (megcount==25)
			{
				megcount = 0;
//				Serial.println("emgwrite");
				SPI.end();
				MySignals.enableSensorUART(BLE);
				MySignals_BLE.writeLocalAttribute(76, valueMEG, 50); //attribute value not correct
//				Serial.println("emgfinishwrite");
				countern++;
				Serial.println(countern);
			}
			if (countern == 10)
			{
				rawtemperature = MySignals.getTemperature() * 100;
//				Serial.print("Temperature (*C): ");
//				Serial.println(rawtemperature);
				rawtemp_low = rawtemperature & 0b0000000011111111;
				rawtemp_high = (rawtemperature & 0b1111111100000000) / 256;
				valuetemperature[0] = rawtemp_high;
				valuetemperature[1] = rawtemp_low;
				SPI.end();
				MySignals.enableSensorUART(BLE);
				MySignals_BLE.writeLocalAttribute(72, valuetemperature, 2);
				countern++;
//				Serial.println("temp");
//				Serial.print("Attribute32");
//				Serial.println(MySignals_BLE.readLocalAttribute(72));
//				Serial.println(MySignals_BLE.attributeValue[0]);
//				Serial.println(MySignals_BLE.attributeValue[1]);
//				Serial.println(MySignals_BLE.attributeValue[2]);
//				Serial.println(MySignals_BLE.attributeValue[3]);
//				Serial.println(MySignals_BLE.attributeValue[4]);
//				Serial.println(MySignals_BLE.attributeValue[5]);
//				Serial.println(MySignals_BLE.attributeValue[6]);
//				Serial.println(MySignals_BLE.attributeValue[7]);
//				Serial.println(MySignals_BLE.attributeValue[8]);
//				Serial.println(MySignals_BLE.attributeValue[9]);
			}
			else if (countern==20)
			{
				MySignals.enableSensorUART(PULSIOXIMETER_MICRO);
				pulsioximeter_state = MySignals.getPulsioximeterMicro();
//				Serial.print("pulsioximeter_state");
//				Serial.println(pulsioximeter_state);
				if (pulsioximeter_state== 1)
				{
					rawPulse = MySignals.pulsioximeterData.BPM;
					rawSPO2 = MySignals.pulsioximeterData.O2;
					rawPulse_low = rawPulse & 0b0000000011111111;
					rawPulse_high = (rawPulse & 0b1111111100000000) / 256;
					rawSPO2_low = rawSPO2 & 0b0000000011111111;
					rawSPO2_high = (rawSPO2 & 0b1111111100000000) / 256;
					valueSPO2[0] = rawPulse_high;
					valueSPO2[1] = rawPulse_low;
					valueSPO2[2] = rawSPO2_high;
					valueSPO2[3] = rawSPO2_low;
					SPI.end();
					MySignals.enableSensorUART(BLE);
					MySignals_BLE.writeLocalAttribute(96, valueSPO2, 4);
					countern = 0;
//					Serial.println("spo2");
				}
				else
				{
					rawPulse = 999;
					rawSPO2 = 999;
					rawPulse_low = rawPulse & 0b0000000011111111;
					rawPulse_high = (rawPulse & 0b1111111100000000) / 256;
					rawSPO2_low = rawSPO2 & 0b0000000011111111;
					rawSPO2_high = (rawSPO2 & 0b1111111100000000) / 256;
					valueSPO2[0] = rawPulse_high;
					valueSPO2[1] = rawPulse_low;
					valueSPO2[2] = rawSPO2_high;
					valueSPO2[3] = rawSPO2_low;
					SPI.end();
					MySignals.enableSensorUART(BLE);
					MySignals_BLE.writeLocalAttribute(96, valueSPO2, 4);
					countern = 0;
//					Serial.println("spo20202");
				}
			}
		}
		else
		{
			delay(1000);
		}
	}
}