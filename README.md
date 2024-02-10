//only chanel 1 AIN 1+ AIN -


#include <SPI.h>
#define CS 10
 
uint8_t val16;
uint8_t receivedVal;
uint8_t kom=0x60;
uint16_t itemp;/*internal temperature*/
uint16_t valit;
uint8_t konf_temp=0b00010000;
uint16_t sel_temp=0b0000000010000110;//0b00001000000010110
uint8_t czyt_temp=0b01011100;

uint16_t value; 
uint16_t value2; 
uint16_t  Code;
uint16_t Code2;
float internaltep;
float AIN;

uint16_t full_Scale_Calibration=0b11001000000001010;//11111000000001010
uint16_t zero_Scale_Calibration=0b11101000000001010;

uint8_t adress_mode_register=0b00001000; //01 011 100
uint8_t adress_config_register=0b00010000;//01011000
uint8_t adress_data_register=0b01011100;//było 0b00011000
uint8_t adress_data_register_single=0b01011000;//było 0b00011000
uint8_t adress_offset_register=0b01110000;
uint8_t adress_full_scale_register=0b01111000;
/*uint8_t adress_mode_register=0b10001000; //01 0001 000
uint8_t adress_config_register=0b1001000;//01010000
uint8_t adress_data_register=0b01011000;//było 0b00011000*/
uint8_t data_register_reset=0x0000;
uint16_t mode_register=0b0000000000000001;//ma być 0b0000000000000001;
uint16_t mode_register_single_conversation=0b0010000000000001;//ma być 0b0000000000000001;
//uint16_t config_register=0b0000000000000000;//0b0001000000000000;uniporal 0b0000000000000000;biporal
uint16_t config_register_unipolar_temp=0b00001000000000110;//0b0001000000000000;uniporal 0b0000000000000000;biporal
//uint16_t sel_temp=0b00001000000010110;
uint16_t idle_register=0b0100000000000001;
uint16_t power_down_register=0b001100000000001;
//uint16_t config_register=0x86;//0b0100000000010000;//configuration register gain 128 0101011100000000 internal external 00
uint16_t data_register;//=0b0000000001011000;//0b0000000010000000;
uint16_t offset_register;
uint16_t full_scale_register;
uint16_t offset_register_1;
uint16_t full_scale_register_1;

const int GAIN=128;
uint16_t config_registe_gain=0b0000011100000000;
uint16_t config_register_unipolar_temp_gain=0b00001111000000110;

void id() {
  digitalWrite(CS, LOW);
  SPI.transfer(0x8);
  SPI.transfer16(0x500A);
  digitalWrite(CS, HIGH);
  delay(5);
  digitalWrite(CS, LOW);
  SPI.transfer(kom);
  delay(3);
  uint8_t receivedVal = SPI.transfer(val16);
  Serial.print('\n');
  Serial.print("id to: ");
  Serial.print(receivedVal, HEX);
  Serial.print('\n');
  delay(100);
  digitalWrite(CS, HIGH);
  delay(100);
}

void internal_temperature() {
    digitalWrite(CS, LOW);
    SPI.transfer16(0xFFFF);
    SPI.transfer16(0xFFFF);
    digitalWrite(CS, HIGH); // Assuming it's HIGH rather than 1

    delay(10);
    digitalWrite(CS, LOW);
    SPI.transfer(0x8);
    SPI.transfer16(0x100A);
    digitalWrite(CS, HIGH); // Assuming it's HIGH rather than 1
    digitalWrite(CS, LOW);
    SPI.transfer(konf_temp);
    SPI.transfer16(config_register_unipolar_temp);
    digitalWrite(CS, HIGH); // Assuming it's HIGH rather than 1
    digitalWrite(CS, LOW);
    SPI.transfer(adress_data_register_single);

    do {
       // Serial.print("szczeka temp");
        //Serial.print('\t');
    } while (digitalRead(MISO) == HIGH);

    uint16_t Code = SPI.transfer16(valit);
    Serial.print("Code to: ");
    Serial.print(Code, HEX);
    Serial.print('\n');

    float AIN = (Code ) * 1.17/2/ ((pow(2, 15) - 1) * 1);
    float internaltep = ((AIN / 0.00081) - 273.15);
    Serial.println("Temperature:");
    Serial.println(internaltep, 2);

    delay(1000);
    digitalWrite(CS, HIGH); // Assuming it's HIGH rather than 1
}

void full_Scale_Calibration_function() {
    digitalWrite(CS, LOW);
    while (digitalRead(MISO) == LOW);
    SPI.transfer(adress_mode_register);
    delay(100);
    SPI.transfer16(full_Scale_Calibration);
    delay(100);
    Serial.println("full_Scale_Calibration");
    digitalWrite(CS, HIGH);
}
void offset_register_and_full_scale_function() {
    digitalWrite(CS, LOW);

    // Configuration for data register
    SPI.transfer(adress_data_register_single);
    SPI.transfer16(mode_register_single_conversation);
    digitalWrite(CS, HIGH);

    // Configuration for config register
    digitalWrite(CS, LOW);
    SPI.transfer(adress_config_register);
    SPI.transfer16(config_registe_gain);
    digitalWrite(CS, HIGH);

    // Configuration for offset register
    digitalWrite(CS, LOW);
    SPI.transfer(adress_offset_register);
    do{}
    while (digitalRead(MISO) == HIGH); // Wait for MISO to go low
    uint16_t offset_register_1 = SPI.transfer16(offset_register);
    Serial.print("offset_register ");
    Serial.print('\t');
    Serial.print(offset_register_1, HEX);
    Serial.print('\n');
    digitalWrite(CS, HIGH);
    digitalWrite (CS,LOW);
SPI.transfer(adress_full_scale_register);
do
{
} while (digitalRead(MISO)==1);

full_scale_register_1= SPI.transfer16(full_scale_register);
Serial.print("full_scale_register ");
Serial.print('\t');
Serial.print(full_scale_register_1,HEX);
Serial.print('\n');
}

void full_scale_register_function() {//not working idk why
    digitalWrite(CS, LOW);

    // Configuration for data register
    SPI.transfer(adress_data_register_single);
    SPI.transfer16(mode_register_single_conversation);
    digitalWrite(CS, HIGH);

    // Configuration for config register
    digitalWrite(CS, LOW);
    SPI.transfer(adress_config_register);
    SPI.transfer(adress_full_scale_register);
    do{}
    while (digitalRead(MISO) == HIGH); // Wait for MISO to go low
    uint16_t full_scale_register_1 = SPI.transfer16(full_scale_register);
    Serial.print("full_scale_register ");
    Serial.print('\t');
    Serial.print(full_scale_register_1, HEX);
    Serial.print('\n');
    digitalWrite(CS, HIGH);
}
void volt_ch1() {
    digitalWrite(CS, LOW);

    // Configuration for data register
    SPI.transfer(adress_data_register_single);
    SPI.transfer16(mode_register_single_conversation);
    digitalWrite(CS, HIGH);

    // Configuration for config register
    digitalWrite(CS, LOW);
    SPI.transfer(adress_config_register);
    SPI.transfer16(config_registe_gain);
    digitalWrite(CS, HIGH);

    // Read voltage from data register
    digitalWrite(CS, LOW);
    SPI.transfer(adress_data_register);

    do {
        //Serial.print("czeka wolt");
        //Serial.print('\t');
    } while (digitalRead(MISO) == HIGH);

    uint16_t value = SPI.transfer16(data_register);
    Serial.print("value voltage");
    Serial.print('\t');
    Serial.print(value, HEX);
    Serial.print('\t');
    Serial.print(" ; Voltage[V]=");
    //float voltage = ((float(value) - (32768+25361*0)*0 )/ (pow(2, 15) - 1) * 5/GAIN); //Output in Voltage biporal
    float voltage = ((((float(value)-21659)*10/128))*0.001-2.5*0)*2; //Output in Voltage biporal
    Serial.print(voltage);
    Serial.print('\n');
    digitalWrite(CS, HIGH);
}
// Example of calling the function


void setup() {
  // put your setup code here, to run once:
   Serial.begin(2400,SERIAL_8E2);
  SPI.begin();
 
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  digitalWrite(CS, LOW);
  SPI.transfer16(0xFFFF);
  SPI.transfer16(0xFFFF);
  digitalWrite(CS,1);
  id();
  internal_temperature();
}

void loop() {
  // put your main code here, to run repeatedly:
//full_Scale_Calibration_function();nie może być dla gain 128
internal_temperature();
offset_register_and_full_scale_function();
//full_scale_register_function();
volt_ch1();
}
