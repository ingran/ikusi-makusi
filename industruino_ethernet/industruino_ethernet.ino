/*
  _____                             
 |_   _|                            
   | |  _ __   __ _ _ __ __ _ _ __  
   | | | '_ \ / _` | '__/ _` | '_ \ 
  _| |_| | | | (_| | | | (_| | | | |
 |_____|_| |_|\__, |_|  \__,_|_| |_|
               __/ |                
              |___/                 
						Engineering

Written by EA2EGA and RDCH106

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

------------------------------------------------------------------------

About the program:

This porgram has been designed for unisg in an Industruino D21G Controller,
with Ind I/O board and a Ethernet Expansion module.
If you want to use all the functions of the Software you should use a SD card.
At the first boot the EEPROM variables will be burnt with the default options.
The default network setting is DHCP, so the it will try to aqquire a IP from
a DHCP server. If it fails to do so in 60 seconds, it will load the 192.168.1.177/24.
Anyway you will be able to see the IP in the screen "Red", at this point you can
access via a web browser and configure all parameters.

Requeriments / Dependencies:

For an adecuate DHCP Timeout, replace timeout values in the file \Ethernet2\src\Dhcp.cpp:
int beginWithDHCP(uint8_t *, unsigned long timeout = 3000, unsigned long responseTimeout = 2000);  

Screen Number types:

0-14: Status Screens (System Health, Netoworking, Modbus status, Remoto, Alarm Historic, RPi messages, Info Messages)
15-19: Power Analyzer
20-23: Industruino Analog In
24-25: Industruino Analog Out
26-33: Industruino Digital I/O
40-49: Overdigit Extender.
50-53: TUF-2000 Flowmeters
54-57: Temperature meters
58-80: Remote Sensors

Screen types:
Analog Inputs:
	0=Off
	1=Pres (Bar)
	2=Pres (m.c.a.)
	3=Pres (kg/cm3)
	4=Nivel (Metros)
	5=Caudal (m3/h)
	6=Caudal (l/s)
	7=Caudal (l/h)
	8=Caudal (m3/s)
	9=mA (mA)
	10=Cond (uS/cm)
	11=Cond (mS/cm)
	12=Cond (S/cm)
	13=Ph (Sin Unidades)
	14=Temp (ºC)
	15=N/A
	35=Caudal (m3/h) - Medida de pulsos de Optoacoplador
	36=Caudal (l/s) - Medida de pulsos de Optoacoplador
	37=Caudal (l/h) - Medida de pulsos de Optoacoplador
	38=Caudal (m3/s) - Medida de pulsos de Optoacoplador

Analog Output:
	
Digital I/O:
	0=Disabled
	1=Input
	2=Output
	35=Caudal (m3/h) - Medida de pulsos reed 
	36=Caudal (l/s) - Medida de pulsos reed 
	37=Caudal (l/h) - Medida de pulsos reed
	38=Caudal (m3/s) - Medida de pulsos reed
	
*/
#define SERIALN_BUFFER_SIZE 256	//Is is used for Modbus RS-485 perform better
#define WEBDUINO_COMMANDS_COUNT 9

#include "U8glib.h"				   //Required for graphic Screen
#include <Indio.h>				   //Required for Industruino Industrial I/O
#include <SPI.h>                 //Required fot multiple libraries
#include <Ethernet2.h>           //Required for D21G Ethernet
#include <UC1701.h>			      //Required for Industruino Display
#include <SimpleModbusMaster.h>	//Required for RS-485 Modbus
#include <I2C_eeprom.h>          //Required for EEPROM in D21G
#include <EEPROM.h>          //Required for EEPROM in D21G
#define EEPROM_SIZE 255
#define W5500_ETHERNET_SHIELD	   //Required for Industruino Ethernet
#include <Adafruit_SleepyDog.h>  //Required for Watchdog Timer
                                 // also need to install https://github.com/adafruit/Adafruit_ASFcore
#include <SD.h>                  //Required for SD card
#include <Wire.h>                //Required for multiple libraries
#include <MCP7940.h>             //Required for RTC

#define DEBUG

#define EASTRON_WATT //If Eastron SDM630M-CT returns power in watts (instead of kilowatts), enable the flag

//Memory Free
#include <MemoryFree.h>

//WebDuino Params
#define WEBDUINO_FAIL_MESSAGE "<h1>Request Failed</h1>"
#include "SPI.h" // new include
#include "avr/pgmspace.h" // new include
#include "WebServer.h"
const byte ETH_CS1 = 10; //chip select 1

//FRAM params
const byte CMD_WREN = 0x06; //0000 0110 Set Write Enable Latch
const byte CMD_WRDI = 0x04; //0000 0100 Write Disable
const byte CMD_RDSR = 0x05; //0000 0101 Read Status Register
const byte CMD_WRSR = 0x01; //0000 0001 Write Status Register
const byte CMD_READ = 0x03; //0000 0011 Read Memory Data
const byte CMD_WRITE = 0x02; //0000 0010 Write Memory Data
const byte FRAM_CS1 = 6; //chip select 1
byte fram_time;
unsigned long reboot_count;

//SD params
const byte SD_CS1 = 4; //chip select 1
byte SDOK=0;
Sd2Card card;
SdVolume volume;

#define sw_ver "0.91"

#define Industruino_logo_width 128
#define Industruino_logo_height 64
static unsigned char Industruino_logo_bits[] U8G_PROGMEM = {
   0xff, 0x9f, 0x01, 0x04, 0x00, 0x30, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x0b, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x19, 0xdf,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf7, 0x00, 0x00, 0x00,
   0x00, 0xc0, 0x39, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xf7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x17, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xfb, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x70, 0xfc, 0x01, 0x0e, 0x3c, 0x1c, 0xe0, 0x43, 0x1e, 0xf8,
   0x03, 0x00, 0x00, 0x00, 0x00, 0x20, 0x10, 0xfc, 0x01, 0x0e, 0x1c, 0x18,
   0xe0, 0x00, 0x1e, 0xf8, 0x03, 0x18, 0xf0, 0xff, 0xcf, 0x0f, 0xdf, 0xf8,
   0x73, 0x1e, 0x8e, 0x39, 0xe7, 0x3c, 0x3e, 0xfc, 0x13, 0x18, 0x10, 0x00,
   0x0e, 0x0f, 0x50, 0xf8, 0xf3, 0x5f, 0xce, 0x39, 0x7f, 0x7e, 0xbe, 0xfc,
   0x13, 0x18, 0x10, 0x00, 0x8f, 0x0f, 0x50, 0xf8, 0xc3, 0xcf, 0x7c, 0x3c,
   0x7c, 0xfe, 0x9f, 0xf9, 0x13, 0x18, 0x10, 0x00, 0x8f, 0x0f, 0x70, 0xf8,
   0xc3, 0x0f, 0x3c, 0x3e, 0x7c, 0x1e, 0x1c, 0xf8, 0x13, 0x1c, 0x10, 0x04,
   0x84, 0x0f, 0x70, 0xf8, 0xf3, 0x0f, 0x9c, 0x3f, 0x7f, 0x1e, 0x1c, 0xf8,
   0x1b, 0x0c, 0xbc, 0xbf, 0x9f, 0xff, 0x7f, 0xf0, 0x73, 0xe6, 0xc9, 0x39,
   0xe7, 0x78, 0xce, 0xf3, 0x0b, 0x0c, 0x3c, 0xf9, 0x9f, 0xff, 0x3f, 0xf0,
   0x01, 0xc2, 0x00, 0x18, 0xe0, 0x00, 0x86, 0xe1, 0x0b, 0x0c, 0x78, 0x00,
   0x86, 0x63, 0x10, 0xf0, 0x01, 0xc2, 0x00, 0x18, 0xe0, 0x83, 0x87, 0xe1,
   0x0b, 0x0c, 0x04, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x0b, 0x0c, 0x00, 0xe2, 0x00, 0x00, 0x00, 0xe0,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0b, 0x0c, 0xfc, 0xff,
   0xff, 0xff, 0xf9, 0xe7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0x09, 0x0c, 0xfe, 0xff, 0xff, 0xff, 0xaf, 0xef, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xdf, 0xcf,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0xfc, 0xff,
   0xff, 0xff, 0xff, 0xc7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0x03, 0x00, 0xfc, 0x83, 0xff, 0xff, 0x7f, 0xc0, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x01, 0x40, 0xfc, 0x80, 0xff, 0xff, 0x1f, 0xc0,
   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x40, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0xfe, 0xff, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x40, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x0e, 0x00,
   0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xfe, 0x1f, 0x00, 0x00, 0xfe, 0xff, 0xfc, 0xef, 0xff, 0xff, 0xff,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x0f, 0x00, 0x00, 0x8c, 0xff, 0x1d,
   0x3f, 0xfc, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xfd, 0x1f, 0x00,
   0x00, 0xdc, 0x70, 0xbc, 0xd7, 0xf9, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
   0xf0, 0xfd, 0xef, 0x03, 0x00, 0xdc, 0xad, 0xbd, 0xd7, 0xfb, 0xff, 0xff,
   0x00, 0x00, 0x00, 0x60, 0xc3, 0xff, 0xff, 0x83, 0x1d, 0xdc, 0xad, 0xbd,
   0xdf, 0xfb, 0xff, 0xff, 0x00, 0x00, 0x00, 0x10, 0xe4, 0xfd, 0xef, 0x81,
   0x18, 0xdc, 0xad, 0xbd, 0xdb, 0xf9, 0xff, 0xff, 0x18, 0x00, 0x00, 0x00,
   0xc4, 0xff, 0xff, 0x01, 0x10, 0x8c, 0x48, 0x18, 0x3b, 0xfc, 0xff, 0xff,
   0x18, 0x00, 0x0c, 0x00, 0x84, 0xbd, 0xff, 0x01, 0x1d, 0xfc, 0xff, 0xff,
   0xfd, 0xff, 0xff, 0xff, 0x18, 0x00, 0x1e, 0x00, 0x94, 0xff, 0xff, 0x83,
   0x1f, 0xfc, 0xff, 0xff, 0xfd, 0xff, 0xff, 0xff, 0x1c, 0x00, 0x1e, 0x10,
   0x96, 0xfd, 0xff, 0x87, 0x7f, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0x1c, 0x00, 0x1e, 0x20, 0xf3, 0xff, 0xff, 0x87, 0x1d, 0xfc, 0x7f, 0xde,
   0xff, 0xff, 0xff, 0xff, 0x2c, 0x00, 0x0e, 0x00, 0xf0, 0xff, 0xff, 0x03,
   0x00, 0xfc, 0xbf, 0xed, 0xff, 0xff, 0xff, 0xff, 0x2c, 0x00, 0x0e, 0x00,
   0x00, 0xf8, 0x07, 0x00, 0x00, 0xbc, 0xb3, 0xf5, 0xff, 0xff, 0xff, 0xff,
   0x3e, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xc0, 0x0f, 0x00, 0x9c, 0x6d, 0xc6,
   0xff, 0xff, 0xff, 0xff, 0x6e, 0x00, 0x3e, 0x00, 0xb0, 0xff, 0xdf, 0x0f,
   0x00, 0xbc, 0xaf, 0xb5, 0xff, 0xff, 0xff, 0xff, 0x7e, 0x00, 0x3e, 0x00,
   0x70, 0x0f, 0x8c, 0x1f, 0x00, 0xbc, 0xb7, 0xb5, 0xff, 0xff, 0xff, 0xff,
   0x7e, 0x00, 0x37, 0x00, 0x40, 0x00, 0x10, 0x00, 0x80, 0x19, 0x61, 0xce,
   0xff, 0xff, 0xff, 0xff, 0xee, 0x03, 0xb7, 0xff, 0xff, 0xff, 0x7f, 0x00,
   0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xee, 0xff, 0xf7, 0xef,
   0xff, 0xff, 0x7f, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xee, 0xff, 0x97, 0xff, 0xff, 0xff, 0x7f, 0x00, 0x80, 0xf8, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0xce, 0xff, 0xb7, 0xff, 0xff, 0xff, 0xff, 0xff,
   0x7f, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0c, 0xff, 0xb7, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x7f, 0x18, 0x51, 0x84, 0xff, 0xff, 0xff, 0xff,
   0x40, 0xff, 0xbd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x68, 0xdb, 0x6e,
   0x67, 0x72, 0x1f, 0xe7, 0xc0, 0x3f, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xff, 0xc8, 0xab, 0x6e, 0xdb, 0x3a, 0xef, 0xde, 0x00, 0x3f, 0xc0, 0xff,
   0xff, 0x0f, 0xe0, 0xff, 0x7f, 0x38, 0xa7, 0x8e, 0xc3, 0x7a, 0xef, 0xc6,
   0x00, 0x3f, 0xc0, 0xff, 0xe7, 0x0f, 0xe0, 0xff, 0xff, 0x68, 0x77, 0x2f,
   0xfb, 0x7d, 0xef, 0xda, 0x80, 0x3f, 0xe4, 0xff, 0xc3, 0x07, 0xe0, 0xf3,
   0xff, 0x88, 0x77, 0x47, 0xe6, 0x3d, 0x1a, 0x87, 0x80, 0x3f, 0xc4, 0x1f,
   0x00, 0x00, 0xc0, 0xf3, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0xc0, 0x3f, 0xc4, 0x1f, 0x00, 0x00, 0x00, 0xf0, 0x7f, 0xf8, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xe4, 0x1f, 0x00, 0x00, 0x00, 0xf0,
   0x7f, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xe0, 0x1f,
   0x00, 0x00, 0x00, 0xf0, 0x7f, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0x00, 0x00, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0xf0, 0x7f, 0xf8, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xf8, 0x1f, 0x00, 0x00, 0x00, 0xe0,
   0x7f, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xf8, 0x0f,
   0x00, 0x00, 0x00, 0xc0, 0x7f, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0xf8, 0xff, 0xff,
   0xff, 0xff, 0xff, 0xff };

U8GLIB_MINI12864 u8g(21, 20, 19, 22);

P(Page_start10) = "<html><head><title>Monitorization System - S/N: ";
P(Page_start11) = "</title>";

P(Page_start_css) = "<link rel='icon' href='' sizes='32x32' />"
"<style type='text/css'>"
"html{font-family:sans-serif}body{margin:0}article,aside,details,figcaption,figure,footer,header,hgroup,main,menu,nav,section,summary{display:block}a{background-color:transparent}a:active,a:hover{outline:0}b,strong{font-weight:700}dfn{font-style:italic}h1{margin:0.67em 0;font-size:2em}mark{color:rgb(0,0,0);background:rgb(255,255,0)}small{font-size:80%}sub,sup{position:relative;font-size:75%;line-height:0;vertical-align:baseline}sup{top:-0.5em}sub{bottom:-0.25em}img{border:0}svg:not(:root){overflow:hidden}figure{margin:1em 40px}hr{height:0;box-sizing:content-box}pre{overflow:auto}"
"button,input,optgroup,select,textarea{margin:0;font-style:inherit;font-variant:inherit;font-weight:inherit;font-stretch:inherit;font-size:inherit;line-height:inherit;font-family:inherit;color:inherit}button{overflow:visible}button,select{text-transform:none}button,html input[type='button'],input[type='reset'],input[type='submit']{-webkit-appearance:button;cursor:pointer}button[disabled],html input[disabled]{cursor:default}input{line-height:normal}fieldset{padding:0.35em 0.625em 0.75em;margin:0 2px;border:1px solid silver}legend{padding:0;border:0}textarea{overflow:auto}optgroup{font-weight:700}table{border-spacing:0;border-collapse:collapse}td,th{padding:0}*{box-sizing:border-box}::after,::before{box-sizing:border-box}"
"html{font-size:10px;-webkit-tap-highlight-color:rgba(0,0,0,0)}body{font-family:'Helvetica Neue',Helvetica,Arial,sans-serif;font-size:14px;line-height:1.42857;color:rgb(51,51,51);background-color:rgb(255,255,255)}button,input,select,textarea{font-family:inherit;font-size:inherit;line-height:inherit}a{color:rgb(51,122,183);text-decoration:none}a:focus,a:hover{color:rgb(35,82,124);text-decoration:underline}a:focus{outline:-webkit-focus-ring-color auto 5px;outline-offset:-2px}figure{margin:0}img{vertical-align:middle}"
".h1,.h2,.h3,h1,h2,h3{font-family:inherit;font-weight:500;line-height:1.1;color:inherit}.h1,.h2,.h3,h1,h2,h3{margin-top:20px;margin-bottom:10px}.h1,h1{font-size:36px}.h2,h2{font-size:30px}.h3,h3{font-size:24px}hr{margin-top:20px;margin-bottom:20px;border-width:1px 0 0;border-top-style:solid;border-top-color:rgb(238,238,238)}p{margin:0 0 10px}ol,ul{margin-top:0;margin-bottom:10px}ol ol,ol ul,ul ol,ul ul{margin-bottom:0}.container{padding-right:15px;padding-left:15px;margin-right:auto;margin-left:auto}@media (min-width:768px){.container{width:750px}}@media (min-width:992px){.container{width:970px}}@media (min-width:1200px){.container{width:1170px}}"
"table{background-color:transparent}caption{padding-top:8px;padding-bottom:8px;color:rgb(119,119,119);text-align:left}th{text-align:left}.table{width:100%;max-width:100%;margin-bottom:20px}.table > tbody > tr > td,.table > tbody > tr > th,.table > tfoot > tr > td,.table > tfoot > tr > th,.table > thead > tr > td,.table > thead > tr > th{padding:8px;line-height:1.42857;vertical-align:top;border-top-width:1px;border-top-style:solid;border-top-color:rgb(221,221,221)}.table > thead > tr > th{vertical-align:bottom;border-bottom-width:2px;border-bottom-style:solid;border-bottom-color:rgb(221,221,221)}"
".table > caption + thead > tr:first-child > td,.table > caption + thead > tr:first-child > th,.table > colgroup + thead > tr:first-child > td,.table > colgroup + thead > tr:first-child > th,.table > thead:first-child > tr:first-child > td,.table > thead:first-child > tr:first-child > th{border-top-width:0}.table > tbody + tbody{border-top-width:2px;border-top-style:solid;border-top-color:rgb(221,221,221)}.table .table{background-color:rgb(255,255,255)}.table-hover > tbody > tr:hover{background-color:rgb(245,245,245)}.table-responsive{min-height:0.01%;overflow-x:auto}"
"fieldset{min-width:0;padding:0;margin:0;border:0}legend{display:block;width:100%;padding:0;margin-bottom:20px;font-size:21px;line-height:inherit;color:rgb(51,51,51);border-width:0 0 1px;border-bottom-style:solid;border-bottom-color:rgb(229,229,229)}label{display:inline-block;max-width:100%;margin-bottom:5px;font-weight:700}select[multiple],select[size]{height:auto}output{display:block;padding-top:7px;font-size:14px;line-height:1.42857;color:rgb(85,85,85)}"
".form-control{display:block;width:100%;height:34px;padding:6px 12px;font-size:14px;line-height:1.42857;color:rgb(85,85,85);border:1px solid rgb(204,204,204);border-radius:4px;box-shadow:rgba(0,0,0,0.0745098) 0 1px 1px inset;transition:border-color 0.15s ease-in-out,box-shadow 0.15s ease-in-out;background-image:none;background-color:rgb(255,255,255)}.form-control:focus{border-color:rgb(102,175,233);outline:0;box-shadow:rgba(0,0,0,0.0745098) 0 1px 1px inset,rgba(102,175,233,0.6) 0 0 8px}.form-control::-webkit-input-placeholder{color:rgb(153,153,153)}textarea.form-control{height:auto}"
".btn{display:inline-block;padding:6px 12px;margin-bottom:0;font-size:14px;font-weight:400;line-height:1.42857;text-align:center;white-space:nowrap;vertical-align:middle;touch-action:manipulation;cursor:pointer;-webkit-user-select:none;border:1px solid transparent;border-radius:4px;background-image:none}.btn.active.focus,.btn.active:focus,.btn.focus,.btn:active.focus,.btn:active:focus,.btn:focus{outline:-webkit-focus-ring-color auto 5px;outline-offset:-2px}.btn.focus,.btn:focus,.btn:hover{color:rgb(51,51,51);text-decoration:none}.btn.active,.btn:active{outline:0;box-shadow:rgba(0,0,0,0.121569) 0 3px 5px inset;background-image:none}.btn.disabled,.btn[disabled],fieldset[disabled] .btn{cursor:not-allowed;box-shadow:none;opacity:0.65}a.btn.disabled,fieldset[disabled] a.btn{pointer-events:none}"
".btn-primary{color:rgb(255,255,255);border-color:rgb(46,109,164);background-color:rgb(51,122,183)}.btn-primary.focus,.btn-primary:focus{color:rgb(255,255,255);border-color:rgb(18,43,64);background-color:rgb(40,96,144)}.btn-primary:hover{color:rgb(255,255,255);border-color:rgb(32,77,116);background-color:rgb(40,96,144)}.btn-primary.active,.btn-primary:active,.open > .dropdown-toggle.btn-primary{color:rgb(255,255,255);border-color:rgb(32,77,116);background-color:rgb(40,96,144)}"
".btn-primary.active.focus,.btn-primary.active:focus,.btn-primary.active:hover,.btn-primary:active.focus,.btn-primary:active:focus,.btn-primary:active:hover,.open > .dropdown-toggle.btn-primary.focus,.open > .dropdown-toggle.btn-primary:focus,.open > .dropdown-toggle.btn-primary:hover{color:rgb(255,255,255);border-color:rgb(18,43,64);background-color:rgb(32,77,116)}.btn-primary.active,.btn-primary:active,.open > .dropdown-toggle.btn-primary{background-image:none}.btn-primary.disabled.focus,.btn-primary.disabled:focus,.btn-primary.disabled:hover,.btn-primary[disabled].focus,.btn-primary[disabled]:focus,.btn-primary[disabled]:hover,fieldset[disabled] .btn-primary.focus,fieldset[disabled] .btn-primary:focus,fieldset[disabled] .btn-primary:hover{border-color:rgb(46,109,164);background-color:rgb(51,122,183)}.btn-primary .badge{color:rgb(51,122,183);background-color:rgb(255,255,255)}"
"select.input-group-lg > .form-control,select.input-group-lg > .input-group-addon,select.input-group-lg > .input-group-btn > .btn{height:46px;line-height:46px}select[multiple].input-group-lg > .form-control,select[multiple].input-group-lg > .input-group-addon,select[multiple].input-group-lg > .input-group-btn > .btn,textarea.input-group-lg > .form-control,textarea.input-group-lg > .input-group-addon,textarea.input-group-lg > .input-group-btn > .btn{height:auto}.input-group-sm > .form-control,.input-group-sm > .input-group-addon,.input-group-sm > .input-group-btn > .btn{height:30px;padding:5px 10px;font-size:12px;line-height:1.5;border-radius:3px}select.input-group-sm > .form-control,select.input-group-sm > .input-group-addon,select.input-group-sm > .input-group-btn > .btn{height:30px;line-height:30px}"
"select[multiple].input-group-sm > .form-control,select[multiple].input-group-sm > .input-group-addon,select[multiple].input-group-sm > .input-group-btn > .btn,textarea.input-group-sm > .form-control,textarea.input-group-sm > .input-group-addon,textarea.input-group-sm > .input-group-btn > .btn{height:auto}.container-fluid > .navbar-collapse,.container-fluid > .navbar-header,.container > .navbar-collapse,.container > .navbar-header{margin-right:-15px;margin-left:-15px}.btn .label{position:relative;top:-1px}.btn .badge{position:relative;top:-1px}.btn-group-xs > .btn .badge,.btn-xs .badge{top:0;padding:1px 5px}.container .jumbotron,.container-fluid .jumbotron{padding-right:15px;padding-left:15px;border-radius:6px}"
".jumbotron .container{max-width:100%}.modal-footer .btn + .btn{margin-bottom:0;margin-left:5px}.modal-footer .btn-group .btn + .btn{margin-left:-1px}.carousel-caption .btn{text-shadow:none}"
".btn-group-vertical > .btn-group::after,.btn-group-vertical > .btn-group::before,.btn-toolbar::after,.btn-toolbar::before,.clearfix::after,.clearfix::before,.container-fluid::after,.container-fluid::before,.container::after,.container::before,.dl-horizontal dd::after,.dl-horizontal dd::before,.form-horizontal .form-group::after,.form-horizontal .form-group::before,.modal-footer::after,.modal-footer::before,.modal-header::after,.modal-header::before,.nav::after,.nav::before,.navbar-collapse::after,.navbar-collapse::before,.navbar-header::after,.navbar-header::before,.navbar::after,.navbar::before,.pager::after,.pager::before,.panel-body::after,.panel-body::before,.row::after,.row::before{display:table;content:" "}"
".btn-group-vertical > .btn-group::after,.btn-toolbar::after,.clearfix::after,.container-fluid::after,.container::after,.dl-horizontal dd::after,.form-horizontal .form-group::after,.modal-footer::after,.modal-header::after,.nav::after,.navbar-collapse::after,.navbar-header::after,.navbar::after,.pager::after,.panel-body::after,.row::after{clear:both}th,td{text-align:center;vertical-align:middle}"
"</style>";

P(Page_start2) = "</head><body>\n<div class='container'>\n";
P(Page_end) = "<p><a href=\"index.html\">Menu Principal</a></p>\n</div>\n</body></html>";
P(Tail_end) = "'<br>\n";
P(Parsed_item_separator) = " = '";
P(Line_break) = "<br>\n";
P(comma) = ",";

P(table_start) = "<div class='table-responsive'>\n<table class='table table-hover'>\n";
P(table_th_start) = "<th>";
P(table_th_end) = "</th>\n";
P(table_tr_start) = "<tr>\n";
P(table_tr_end) = "</tr>\n";
P(table_td_start) = "<td>";
P(table_td_end) = "</td>\n";
P(table_end) = "</table>\n</div>\n";

P(Form_input_text_start) = "<input type=\"text\" name=\"";
P(Form_input_value)  = "\" value=\"";
P(Form_input_size2) = "\" maxlength=\"2\" size=\"2";
P(Form_input_size3) = "\" maxlength=\"3\" size=\"3";
P(Form_input_size10) = "\" size=\"10";
P(Form_input_size32) = "\" size=\"32";
P(Form_input_size5) = "\" size=\"5";
P(Form_input_end_norm) = "\">";
P(Form_input_end_spec) = "\">\n";
P(Form_end) = "</FORM>";
P(Form_input_send) = "<INPUT class='btn btn-primary' type=\"submit\" value=\"Guardar\">";

P(Form_input_radio_start) = "<input type=\"radio\" name=\"";
P(Form_input_end_chedcke) = "\" checked>\n";



P(Index_10) = "<h1>Monitorization System</h1>\n"
"<p>Serial Number: ";
P(Index_11) = " - Software Version: " sw_ver " - Free Ram: ";
P(Index_2) = " Bytes</p>";
P(Index_css_on) = "<p><a href=\"index.html?css=0\">Deshabilitar CSS</a></p>";
P(Index_css_off) = "<p><a href=\"index.html?css=1\">Habilitar CSS</a></p>";
P(Index_3) = "<h3>Menu Principal</h3>\n<ul>\n<li><a href=\"network.html\">Red</a></li>\n<li><a href=\"analog.html\">Entradas Analogicas</a></li>\n<li><a href=\"adam.html\">Extensor Analogico</a></li>\n<li><a href=\"modbus.html\">Modbus</a></li>\n<li><a href=\"remote.html\">Sensores Remotos</a></li>\n<li><a href=\"io.html\">Entradas y Salidas</a></li>\n<li><a href=\"log.html\">Log</a></li>\n<li><a href=\"index.html?default\">Cargar valores por defecto</a></li>\n<li><a href=\"index.html?reboot\">Reiniciar</a></li>\n</ul>\n<h3>Info</h3>\n";
P(Http400) = "HTTP 400 - BAD REQUEST";


P(Analog_head) = "\n<h1>Entradas Analogicas</h1>\n<p><a href=\"analog.html\">REFRESCAR</a></p>\n";
P(Analog_head_calib) = "\n<h2>Valores de calibracion</h2>\n<p>Deben utilizarse dos puntos para calibrar un sensor: X es la entrada en mA y la Y es el valor que le corresponde a esa entrada.<br>Si la entrada está en modo Opto, X1 es el tiempo maximo de medida (5), Y1 es el numero máximo de pulsos (100)y X2 es la constante de pulsos<br>\n<br>\nTipos: ";
P(Analog_head_calib_add) = "\n<h2>Enter New Calib</h2>\n";
P(Analog_pulsos) = "Opto Pulsos - ";
P(Analog_in_text_number) = "Analog In";
P(Analog_in_text_raw) = "RAW";
P(Analog_in_text_ma) = "mA";
P(Analog_in_text_corrected) = "Valor";
P(Analog_in_text_type) = "Tipo";
P(Analog_in_text_offset) = "Offset";
P(Analog_in_text_slope) = "Slope";
P(Analog_in_text_X1) = "X1";
P(Analog_in_text_X2) = "X2";
P(Analog_in_text_Y1) = "Y1";
P(Analog_in_text_Y2) = "Y2";
P(Analog_in_text_save) = "Guardar";
P(Form_analog_start1) = "\n<FORM role='form' name=\"form" ;
P(Form_analog_start2) = "\" action=\"analog.html\" method=\"get\">\n";

P(Analog_alarm_head) = "\n<h2>Alarmas y salidas</h2>\n";
P(Analog_alarm_01) = "Analog In";
P(Analog_alarm_02) = "Habilitado";
P(Analog_alarm_03) = "Mayor(1) Menor(0)";
P(Analog_alarm_04) = "Latching";
P(Analog_alarm_05) = "Esperar";
P(Analog_alarm_06) = "Auto Unlatch";
P(Analog_alarm_07) = "Nivel ON";
P(Analog_alarm_08) = "Delay ON";
P(Analog_alarm_09) = "Nivel OFF";
P(Analog_alarm_10) = "Delay OFF";
P(Analog_alarm_11) = "T minimo OFF";

P(Modbus_head) = "\n<h1>Menu Modbus</h1>\n</p>\n<p><a href=\"modbus.html\">REFRESCAR</a>\n</p>";

P(Modbus_Aparato) = "Aparato";
P(Modbus_ID) = "Modbus ID";
P(Modbus_con_stat) = "Estado de la Conexion";
P(Modbus_req) = "Peticiones";
P(Modbus_suc_req) = "Peticiones correctas";
P(Modbus_total_err) = "Errores totales";
P(Modbus_retries) = "Reintentos";
P(Modbus_tiemeout) = "Timeout";
P(Modbus_habilitar) = "Habilitar";
P(Modbus_valor) = "Valor";
P(Modbus_guardar) = "Guardar";
P(Modbus_illegal_funct) = "\n<br>Illegal Function: ";
P(Modbus_illegal_data_addr) = "\n<br>Illegal Data Address: ";
P(Modbus_illegal_data_value) = "\n<br>Illegal Data Value: ";
P(Modbus_illegal_data_except) = "\n<br>Misc Exceptions: ";
P(Form_modbus_start1) = "\n<FORM role='form' name=\"form" ;
P(Form_modbus_start2) = "\" action=\"modbus.html\" method=\"get\">\n";

P(Modbus_Eastron) = "Analizador";
P(Modbus_TUF) = "TUF-2000";
P(Modbus_Overdigit) = "Extensor";
P(Modbus_Temp) = "Temperatura";

P(adam_head) = "\n<h1>Extensor Analogico</h1>\n<p><a href=\"adam.html\">REFRESCAR</a>\n</p>";
P(adam_head_calib) = "\n<h2>Calib Values</h2>\n<br><p>Must set up two points for calibration:<br>\nX is RAW input and Y is the corresponding value corrected.<br>\nAs a guide 4 RAW is 4mA and 20 is 20mA<br>\nTypes: 0 Disbled, 1 Bar, 2 Metros, 3 m3/h, 4 mA, 5 uS, 6 No Units (PH), 7 No Units (Ain)</p>\n";
P(adam_enabled_text) = "<br>\n<p>Habilitar Extensor, 0 = NO , 1 = SI:</p>";
P(ADAM) = "\n<h2>Enter New Calib</h2>\n";
P(Form_adam_start1) = "\n<FORM role='form' name=\"form" ;
P(Form_adam_start2) = "\" action=\"adam.html\" method=\"get\">\n";

P(Network_setup_head)="\n<h1>Red</h1>\n";
P(MAC) = "\nMAC address: ";
P(IP) = "\nIP address: ";
P(Target_IP) = "\nIP Servidor: ";
P(Own_IP_text) = "\nIP System: ";
P(Gateway_IP) = "\nGateway (Static only): ";
P(Netmask_text) = "\nMascara de Red: ";
P(DNS_text) = "\nDNS: ";
P(Packet_type) = "\nTipo de envio: ";
P(Net_type_txt) = "\nNetwork: ";
P(Target_AKEY) = "\nA key: ";
P(Target_NODE) = "\nNodo: ";
P(Target_PORT) = "\nPuerto: ";
P(Target_APIKEY) = "\nAPI KEY: ";
P(Target_EMON_prefix) = "\nEmonCMS prefix: ";
P(Network_Announce_time) = "\nTiempo de envio en segundos: ";
P(SUBNET) = "\nSubnet: ";
P(GW) = "\nGW address: ";
P(DNS_SERVER) = "\nDNS server: ";
P(Form_network_start1) = "\n<FORM role='form' name=\"form" ;
P(Form_network_start2) = "\" action=\"network.html\" method=\"get\">\n";

P(Log_head)="\n<h1>Log</h1>\nInformacion guardada<br>\n<h3>FRAM</h3>";
P(Log_del_sistema)="\n<h3>Log Del Sistema</h3>";
P(Log_sd)="\n<h3>SD</h3>";
P(Log_reinicios)="Reinicios: ";
P(Log_scroll)="<div style='height:200px;width:50%;border:1px solid #ccc;overflow:auto;'>";
P(Log_div_end)="</div>";
P(Log_sd_error)="error opening: ";


P(IO_setup_head)="\n<h1>Entradas y Salidas</h1>\n<h2>Salidas Analógicas</h2>\n";
P(Analog_out_text) = "Salida Analogica";
P(Analog_out_type) = "Tipo";
P(Analog_out_valor) = "Valor";
P(Analog_cal_cal) = "Cal ";
P(Analog_cal_offset) = "offset";
P(Analog_cal_rampa) = "rampa";
P(IO_digital_head)="\n<h2>I/O Digital</h2>\n";
P(Digital_text) = "Digital";
P(Form_io_start1) = "\n<FORM role='form' name=\"form" ;
P(Form_io_start2) = "\" action=\"io.html\" method=\"get\">\n";

P(Remote_head)="\n<h1>Sensores Remotos</h1>\n<p>Mandar un paquete UDP al puerto 8887, de la siguiente configuración: <i>NºRemoto+58</i>:<i>Valor_del_sensor</i>&40:<i>Tipo</i>&41:<i>Calidad_RF</i><br><br>\nEjemplo, sensor Remoto Nº2, valor: 64.2, RF: -62, tipo: Presión (BAR): 60:64.2&&40:1&41:-62 <br><br>\nEl tipo de sensor se guarda para peticiones sucesivas, por lo que no es necesario mandarlo siempre.</p>\n<h1>Tabla</h1>\n";
P(Remoto_numero) = "Numero";
P(Remoto_tipo) = "Tipo";
P(Remoto_rf) = "RF";
P(Remoto_cont) = "Contador";
P(Remoto_tiempo) = "Hace seg.";
P(Remoto_valor) = "Valor";



P(RAM_1) = "RAM (byte): ";
P(RAM_2) = " free of ";

int countdownMS;	//for the watchdog timer

int rtc[7];			//for the RTC	

uint8_t boot_mcusr;

byte last_cycle_key,current_view=1;
unsigned long last_key_press,last_sync_try,time_temp3,last_screen_change;
byte adc_key=0;
int FSM_status=0;
byte screen_number=0;
byte screen_type[128]={1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,	//Status
1,0,0,0,0};												//Power Analyzer
byte show_eastron=1;

byte css_enabled=1;

const int RESET_PIN = 5;

/* This creates an instance of the webserver.  By specifying a prefix
 * of "", all pages will be at the root of the server. */
#define PREFIX ""
WebServer webserver(PREFIX, 80);
unsigned int ciclos=0;

/* EEPROM
EEPROM dataspace
0x00-0x01: First boot flag, for loading defaults at first boot. If "OK", that means that is OK :) 
0x10-0x13: Ethernet IP Address
0x20-0x23: Traget IP
0x28: AKEY
0x29: Node

0x2A: Announce time in seconds
0x2B: Syncron enable

0x30-0x33: Gateway

0x40-0x41: Modbus packets enabled or not.
0x50-0x8F: Analog In Calib Points for calculating Slope and Offset
0x90-0x94: Analog Input Type:
	0=Off
	1=Bar (Presion)
	2=Metros (Sonda)
	3=m3/h (Caudal)
	4=mA
	5=uS	(Conductividad)
	6=No Units	(Ph)
	7=No Units	(AinNumber)
	
0xA0: Flowmeter Type
0xA1: Pulse Meter Input Pin
0xA2: Pulse Meter Max Time
0xA8: Pulse Meter Max Pulses (int)
0xAA: Liters per pulse
0xAE: Disable flowmeter if power < 5000

//Reed Pulse Meter
0x300-0x31F: Reed pulse constant

//I/O
0xC0: Analog 1 Out type		//0 is disabled, 1 is 0-10 Volt, 2 is 0-20mA
0xC1: Analog 2 Out type
0xC2-0xC9: Digital I/O type (1-8)

0xF0:Adam-4017 Enable
0x100-0x17F Adam-4017 Analog In Calib Points for calculating Slope and Offset
0x180-0x187 Adam-4017 Analog Input Type;

0x320-0x32F Serial Nº
0x330-0x33F Node
0x340-0x37F Apikey
0x380-0x3BF target_addr
0x3C0-0x3FF Emon_prefix

0x200-0x260 Alarm data

0x190 Packet type
0x191 net_type



*/

/* FRAM Map
0x1-0x5: how many boots: unsigned long

0x10-0x2F: 8 Unsigned Longs
Counters for pulse counters

*/

//Begin of ModBus config Parameters

// led to indicate that a Modbus communication error is present
// Not wired in current system
#define connection_error_led 30

//////////////////// Port information ///////////////////
#define baud 9600
#define timeout 1000
#define polling 200 // the scan rate

// If the packets internal retry register matches the set retry count then communication is stopped on that packet. To re-enable the packet you must set the "connection" variable to true.
#define retry_count 10 

// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 9

// END of ModBus parameters

typedef struct  {
  byte habilitado;
  byte mayor;
  byte latching;
  byte esperar;
  byte auto_unlatch;
  float nivelON;
  float delayON;
  float nivelOFF;
  float delayOFF;
  float TminOFF;
} AlarmConf;

AlarmConf Alarmas[4];

unsigned long Alarmas_last_on[4],Alarmas_last_no_OFF[4],Alarmas_last_low_time[4];
byte Alarmas_first_cycle[]={0,0,0,0};
byte Alarmas_out_status[]={0,0,0,0};
byte Alarmas_out_flag[]={0,0,0,0};
byte Alarmas_unlatched_flags[]={0,0,0,0};


//Union Used for Converting ModBus Float32 values to C++ float values. Depending on BigEndian or LowEndian, implement in different way while using it.
union u_tag {	
	float f;
	unsigned short asfloat[2];
} u;

union u_tag2 {	
	unsigned long ul;
	byte asbyte[4];
} u_2;

//Start Variables used for ModBus Comunication

unsigned int connection_status;

enum
{
  PACKET1,
  PACKET2,
  PACKET3,
  PACKET4,
  PACKET5,
  PACKET6,
  PACKET7,
  PACKET8,
  PACKET9,
  PACKET10,
  PACKET11,
  PACKET12,
  // leave this last entry
  TOTAL_NO_OF_PACKETS
};

// Create an array of Packets for modbus_update()
Packet packets[TOTAL_NO_OF_PACKETS];

// Create a packetPointer to access each packet individually. This is not required you can access the array explicitly. E.g. packets[PACKET1].id = 2; This does become tedious though...
packetPointer packet1 = &packets[PACKET1];
packetPointer packet2 = &packets[PACKET2];
packetPointer packet3 = &packets[PACKET3];
packetPointer packet4 = &packets[PACKET4];
packetPointer packet5 = &packets[PACKET5];
packetPointer packet6 = &packets[PACKET6];
packetPointer packet7 = &packets[PACKET7];
packetPointer packet8 = &packets[PACKET8];
packetPointer packet9 = &packets[PACKET9];
packetPointer packet10 = &packets[PACKET10];
packetPointer packet11 = &packets[PACKET11];
packetPointer packet12 = &packets[PACKET12];

// The data from the ModBus Slaves will be stored in the regs arrays
unsigned int regs[20],regs2[20],regs3[8],regs4[20],regs5[8],regs6[8],regs7[8],regs8[8],regs9[2],regs10[2],regs11[2],regs12[2];	//Do NOT attempt to read more than 30 registers at Once

byte modbus_enabled[2]={255,255};
float temperaturas[8]={0,0,0,0,0,0,0,0};
float TUF_flow[4]={0,0,0,0};

//network NB: Pins 10, 11, 12 and 13 are reserved for Ethernet module. 
byte mac[] = { 0xDE, 0xDA, 0xBE, 0xAA, 0xAA, 0xAA };
byte ip[] = { 192, 168, 1, 177 };
byte gateway[] = { 192, 168, 1, 17 };
byte subnet[] = { 255, 255, 255, 0 };
byte dnServer[] = {8, 8, 8, 8};
byte syncron_enable=0;

const int backlightPin = 26; // PWM output pin that the LED backlight is attached to, different in Industruino 32u4

static UC1701 lcd;			//define LCD

byte udp_ip[4] = { 0, 0, 0, 0 };
int PORT_UDP=8888;
byte Akey;
char APIKEY[32+1];
char target_adrs[32+1];
char EMON_prefix[32+1];
byte send_packet_type=0;
byte net_type=0;
byte Node;
char SERIALN[15+1];
char NODE[15+1];

#define INPUT_SIZE 64
char IncomingUDP[INPUT_SIZE+1];  //buffer to hold incoming packet,
char Remote_Line[INPUT_SIZE+1];  //buffer to hold incoming packet,
IPAddress remote_input_ip;
unsigned long last_input_packet_time=0;
float remote_val[32];
int remote_rf[32];
unsigned int remote_packets[32] = {0};
unsigned long remote_last[32];

//Define Valiables for Web Server
//EthernetServer server(80);
String data;

//Define UDP client Socket, used for sending data
EthernetUDP Udp;
EthernetClient clientk;

byte ethOK=0; //Ethernet Status
byte dhcp_en=0;

const byte node_id=10;
unsigned long time1, time_temp2, purge_time,time_temp, timepost=0,  timepost_sd=0;
unsigned long post_interval=10000, display_loop;
float Sensorval1, Sensorval2;

byte display_bottom_status=0; //0 is waiting, 1 is sending, 2 is Sent

byte ain_read=0;

byte SyncOK=0;

byte init_status=0;

float offset[12];
float slope[12];
byte ain_type[12];
const float offset_to_ma=19;
const float slope_to_ma=0.005442;
const float offset_overdigit_ma=0;
const float slope_overdigit_ma=0.001194;

char *strings_numeros[]={"0","1","2",NULL};
char *strings_nosi2[]={"No","Si",NULL};
char *strings_anout[]={"Deshabilitado","0-10 Volt","0-20 mA",NULL};
char *strings_packet_type[]={"UDP v1","UDP v2","EmonCMS",NULL};
char *net_type_drop[]={"DHCP","Static",NULL};
char *strings_dionumeros[]={"0","1","2","35","36","37","38",NULL};
char *strings_diotipos[]={"Deshabilitado","Entrada","Salida","Pulsos-Q(m3/h)","Pulsos-Q(l/s)","Pulsos-Q(l/h)","Pulsos-Q(m3/s)",NULL};

char *myStrings[]={"Deshabilitado","Pres(Bar)", "Pres(m.c.a.)", "Pres(KG/cm3)",
"Nivel(m)", "Q(m3/h)","Q(l/s)", "Q(l/h)", "Q(m3/s)", "mA","Cond(uS/cm)","Cond(mS/cm)","Cond(S/cm)","Ph","Temp(ºC)","N/A",NULL}; 
/* Types
	0=Off
	1=Pres (Bar)
	2=Pres (m.c.a.)
	3=Pres (kg/cm3)
	4=Nivel (m)
	5=Q (m3/h)
	6=Q (l/s)
	7=Q (l/h)
	8=Q (m3/s)
	9=mA (mA)
	10=Cond (uS/cm)
	11=Cond (mS/cm)
	12=Cond (S/cm)
	13=Ph (Sin Unidades)
	14=Temp (ºC)
	15=N/A
	35=Q (m3/h) - Medida de pulsos de Optoacoplador
	36=Q (l/s) - Medida de pulsos de Optoacoplador
	37=Q (l/h) - Medida de pulsos de Optoacoplador
	38=Q (m3/s) - Medida de pulsos de Optoacoplador
*/

// Flowmeter Off

//Pulse Meter variables
float Q_flow_opto[4]={0,0,0,0};
byte opto_round=0;

//Reed counter variables

byte last_reed[8]={0,0,0,0,0,0,0,0};
byte reed_counter[8]={0,0,0,0,0,0,0,0};
unsigned long time_reed_last[8]={0,0,0,0,0,0,0,0};//Try if it can work with int 
float reed_constant[8]={1000,1000,1000,1000,1000,1000,1000,1000};	//1m^3/reed pulse
float Q_flow_reed[8]={0,0,0,0,0,0,0,0};
volatile unsigned int pulse_counter[8];
byte dig_inputs;

//Adam-4017+ Values
byte adam_enabled=1;

//I/O valiables
byte io_type[10];
byte io_digital;
float analog_out[2];
float reed_calib[8];
float anout_offsetV[2];
float anout_offsetI[2];
float anout_slopeV[2];
float anout_slopeI[2];

// Values to post
float ain_values_corrected[12];	//Analog values corrected
float eastron_read[12];			//V1-3,I1-3,P1-3,PF1-3
float adam_read[8];
float Q_flow_485=0;					//Flowmeter Q 485

//FIles
File webFile;
File root;

#define bufferMax 128
int bufferSize;
char buffer[bufferMax];

//Variables needed to store 
int lastSampleI,sampleI;
double lastFilteredI, filteredI;

byte first_loop=0;



#define NAMELEN 32
#define VALUELEN 44

int FRAMWrite(int addr, byte *buf, int count=1)
{


  if (addr > 0x7ff) return -1;

  byte addrMSB = (addr >> 8) & 0xff;
  byte addrLSB = addr & 0xff;

  digitalWrite(FRAM_CS1, LOW);   
  SPI.transfer(CMD_WREN);  //write enable 
  digitalWrite(FRAM_CS1, HIGH);

  digitalWrite(FRAM_CS1, LOW);
  SPI.transfer(CMD_WRITE); //write command
  SPI.transfer(addrMSB);
  SPI.transfer(addrLSB);

  for (int i = 0;i < count;i++) SPI.transfer(buf[i]);

  digitalWrite(FRAM_CS1, HIGH);

  return 0;
}



/**
 * Read from FRAM (assuming 2 FM25C160 are used)
 * addr: starting address
 * buf: pointer to data
 * count: data length. 
 *        If this parameter is omitted, it is defaulted to one byte.
 * returns: 0 operation is successful
 *          1 address out of range
 */
int FRAMRead(int addr, byte *buf, int count=1)
{

  if (addr > 0x7ff) return -1;

  byte addrMSB = (addr >> 8) & 0xff;
  byte addrLSB = addr & 0xff;

  digitalWrite(FRAM_CS1, LOW);

  SPI.transfer(CMD_READ);
  SPI.transfer(addrMSB);
  SPI.transfer(addrLSB);

  for (int i=0; i < count; i++) buf[i] = SPI.transfer(0x00);

  digitalWrite(FRAM_CS1, HIGH);

  return 0;
}

void sendNTPpacket(char* address) {
  // set all bytes in the buffer to 0
  memset(IncomingUDP, 0, INPUT_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  IncomingUDP[0] = 0b11100011;   // LI, Version, Mode
  IncomingUDP[1] = 0;     // Stratum, or type of clock
  IncomingUDP[2] = 6;     // Polling Interval
  IncomingUDP[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  IncomingUDP[12]  = 49;
  IncomingUDP[13]  = 0x4E;
  IncomingUDP[14]  = 49;
  IncomingUDP[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write((const uint8_t*)IncomingUDP, INPUT_SIZE);
  Udp.endPacket();
}

void radio_form_builder(WebServer &server, char *menu[],char *text[], char name[], int size, int selected)
{
	byte select_flag=0;
	if (selected>0&&selected<=size)
	{
		select_flag=1;
	}

	int k=0;
	while (k<size)
	{
		server.print("<input type=\"radio\" name=\"");
		server.print(name);
		server.print("\" value=\"");
		server.print(menu[k]);
		server.print("\"");
		if (select_flag==1&&selected==(k+1))
		{
			server.print(" checked");
		}
		server.print("> " );
		server.print(text[k]);
		server.print("\n" );
		k++;
	}
}

void showall(WebServer &server)
{
	String stringtemp;
	stringtemp = String("info,")+String(SERIALN)+String(",")+String(sw_ver);
	server.println(stringtemp);
	server.printP(Line_break);
	
	server.print("net,");
	byte i=0;
	byte btemp1;
	byte itemp1;
	while (i<4)
	{	
		EEPROM.get( 0x20+i, btemp1 );
		server.print(btemp1);
		if (i==3)
			break;
		server.print(".");
		i++;
	}
	server.printP(comma);
	i=0;
	while (i<4)
	{	
		EEPROM.get( 0x30+i, btemp1 );
		server.print(btemp1);
		if (i==3)
			break;
		server.print(".");
		i++;
	}
	server.printP(comma);
	EEPROM.get( 0x2C, itemp1 );
	server.print(itemp1);
	server.printP(comma);
	EEPROM.get( 0x2B, btemp1 );
	server.print(btemp1);
	server.printP(comma);
	EEPROM.get( 0x190, btemp1 );
	server.print(btemp1);
	server.printP(comma);
	EEPROM.get( 0x28, btemp1 );
	server.print(btemp1);
	server.printP(comma);
	EEPROM.get( 0x340, APIKEY );
	server.print(APIKEY);
	server.printP(comma);
	EEPROM.get( 0x380, target_adrs );
	server.print(target_adrs);
	server.printP(comma);
	EEPROM.get( 0x3C0, EMON_prefix );
	server.print(EMON_prefix);
	server.printP(comma);
	EEPROM.get( 0x330, NODE );
	server.print(NODE);
	server.printP(comma);
	EEPROM.get( 0x2A, btemp1 );
	server.print(btemp1);
	server.printP(Line_break);
	
	
	float ain_temp[4];
	 float ain_corr[4];
	 
	 ain_temp[0]=Indio.analogRead(1); //Read Analog-In (output depending on selected mode)
	 ain_temp[1]=Indio.analogRead(2); //Read Analog-In (output depending on selected mode)
	 ain_temp[2]=Indio.analogRead(3); //Read Analog-In (output depending on selected mode)
	 ain_temp[3]=Indio.analogRead(4); //Read Analog-In (output depending on selected mode)
	 
	 byte l=0;
	 while (l<4)
	 {
		ain_temp[l]=(ain_temp[l]-offset_to_ma)*slope_to_ma;
		ain_corr[l]=(ain_temp[l]-offset[l])*slope[l];
		l++;
	 }
	
	i=0;
	
	while (i<4)
	{	
		server.print("Ain");
		server.print(i+1);
		server.printP(comma);
		server.print(ain_temp[i]);
		server.printP(comma);
		server.print(ain_corr[i]);
		server.printP(comma);
		EEPROM.get( 0x90+i, btemp1 );
		server.print(btemp1);
		server.printP(Line_break);
		i++;
	}
	
	server.printP(Line_break);
	
	return;
}

void drop_form_builder(WebServer &server, char *values[],char *text[], int name, int size, int selected)
{
	byte select_flag=0;
	if (selected>0&&selected<=size)
	{
		select_flag=1;
	}

	server.print("<select name='");
	server.print(name);
	server.print("'>\n");
		
	int k=0;	
	while (k<size)
	{
		server.print("<option value='");
		server.print(values[k]);
		server.print("'");
		if (select_flag==1&&selected==(k+1))
		{
			server.print(" selected");
		}
		server.print("> " );
		server.print(text[k]);
		server.print("</option>\n" );
		k++;
	}
	server.print("</select>\n");
}

void drop_form_builder(WebServer &server,char *text[], int name, int size, int selected)
{
	byte select_flag=0;
	if (selected>0&&selected<=size)
	{
		select_flag=1;
	}

	server.print("<select name='");
	server.print(name);
	server.print("'>\n");
		
	int k=0;	
	while (k<size)
	{
		server.print("<option value='");
		server.print(k);
		server.print("'");
		if (select_flag==1&&selected==(k+1))
		{
			server.print(" selected");
		}
		server.print("> " );
		server.print(text[k]);
		server.print("</option>\n" );
		k++;
	}
	server.print("</select>\n");
}

void printDirectory(WebServer &server,File dir, int numTabs, byte tipo )
{
	dir.rewindDirectory();
	while (true) {

		File entry =  dir.openNextFile();
		if (! entry) {
		  // no more files
		  //server.print("No More files");
		  break;
		}
		for (uint8_t i = 0; i < numTabs; i++) {
		  server.print('-');
		}
		server.print("<a href='log.html?download=");		
		server.print(entry.name());
		server.print("'>");
		server.print(entry.name());
		server.print("</a>");
		if (entry.isDirectory()) {
		  server.println("/");
		  printDirectory(server, entry, numTabs + 1,1);
		} else {
		  // files have sizes, directories do not
		  server.print(" - ");
		  server.println(entry.size(), DEC);
		  server.println(" Bytes<br>");
		}
		entry.close();
	}
}


void indexHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) //indexHTML() function This function is used to send the index.html to the client.
{
  URLPARAM_RESULT rc;
  char name[NAMELEN];
  char value[VALUELEN];
  boolean params_present = false;
  byte param_number = 0;
  
  // this line sends the standard "we're all OK" headers back to the browser
  server.httpSuccess();

  // if we're handling a GET or POST, we can output our data here. For a HEAD request, we just stop after outputting headers.
  if (type == WebServer::HEAD)
    return;
    
	// check for parameters
  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, NAMELEN, value, VALUELEN);
      if (rc != URLPARAM_EOS) {
        params_present=true;
        
		if (strcmp(name, "showall") == 0) 
		{
		    showall(server);
			return;
        }
		
		#ifdef DEBUG
		// debug output for parameters
		server.print(name);
        server.printP(Parsed_item_separator);
        server.print(value);
        server.printP(Tail_end);
		#endif
		
		/*
		0xA0: Flowmeter Type
		0xA1: Pulse Meter Input Pin
		0xA2: Pulse Meter Max Time
		0xA8: Pulse Meter Max Pulses (int)
		0xAA: Liters per pulse
		*/
		
 		
		// Analog output type and Digital I/O type
		if (strcmp(name, "css") == 0) 
		{
		    byte out3 = atoi(value);
			#ifdef DEBUG
			// debug output for parameters
			server.print(param_number);
			server.printP(Parsed_item_separator);
			server.print(out3);
			server.printP(Tail_end);
			#endif
		    css_enabled=out3;
        }
		
		if (strcmp(name, "default") == 0) 
		{
		    
			#ifdef DEBUG
			// debug output for parameters
			server.print(param_number);
			server.printP(Parsed_item_separator);
			server.printP(Tail_end);
			#endif
		    burnDefaults();
        }
		
		if (strcmp(name, "reboot") == 0) 
		{
		    
			#ifdef DEBUG
			// debug output for parameters
			server.print(param_number);
			server.printP(Parsed_item_separator);
			server.printP(Tail_end);
			#endif
		    int o;
			countdownMS = Watchdog.enable(500);
			while (1)
			{
				o=0;
			} 
        }
		
		if (strcmp(name, "serial") == 0) 
		{
		    byte i=0;
			while (i<15)
				{
					EEPROM.put( 0x320+i, value[i]);
					i++;
				}
			EEPROM.put( 0x320+i, 0);
        }
		
		load_eeprom();
      }
    }
  }

	server.printP(Page_start10);
	server.print(Serial);
	server.printP(Page_start11);
	if (css_enabled==1)
		server.printP(Page_start_css);
	server.printP(Page_start2);
  
  server.printP(Index_10);
  server.print(SERIALN);
  server.printP(Index_11);
  server.print(freeMemory());
  server.printP(Index_2);
  if (css_enabled==1)
	  server.printP(Index_css_on);
  else
	  server.printP(Index_css_off); 
  
  
  server.printP(Index_3);
  
  /*server.print("<FORM role='form' name='form1' action='analog.html' method='get'>\n");
  radio_form_builder(server, myStrings,myStrings,"tipo", 14, 4);
  server.print("</FORM>\n");
  */
  
  server.print("<p>Reinicios: ");
  server.print(reboot_count);
  server.println(" - ");
  
	long days=0;
	long hours=0;
	long mins=0;
	long secs=0;
	long currentmillis=millis();
	secs = currentmillis/1000; //convect milliseconds to seconds
	mins=secs/60; //convert seconds to minutes
	hours=mins/60; //convert minutes to hours
	days=hours/24; //convert hours to days
	secs=secs-(mins*60); //subtract the coverted seconds to minutes in order to display 59 secs max 
	mins=mins-(hours*60); //subtract the coverted minutes to hours in order to display 59 minutes max
	hours=hours-(days*24); //subtract the coverted hours to days in order to display 23 hours max
	//Display results
	server.print("UpTime: ");
	if (days>0) // days will displayed only if value is greater than zero
	{
	server.print(days);
	server.print(" days and :");
	}
	server.print(hours);
	server.print(":");
	server.print(mins);
	server.print(":");
	server.print(secs);
	server.print("</p>\n");
  
  server.print("\n<p>Tarjeta SD: ");
  switch(card.type()) {
    case SD_CARD_TYPE_SD1:
      server.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      server.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      server.println("SDHC");
      break;
    default:
      server.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    server.println("Partition Error");
    return;
  }


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  server.print(" - FAT");
  server.println(volume.fatType(), DEC);
  server.println();
  
  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  server.print(" - ");
  volumesize /= 1024;
  server.println(volumesize);
  server.print(" KB</p>");
  
  server.printP(Page_end); 
}



void ioPage(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  URLPARAM_RESULT rc;
  char name[NAMELEN];
  char value[VALUELEN];
  boolean params_present = false;
  byte param_number = 0;

  /* this line sends the standard "we're all OK" headers back to the
     browser */
  server.httpSuccess();

  /* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
  if (type == WebServer::HEAD)
    return;

	server.printP(Page_start10);
	server.print(Serial);
	server.printP(Page_start11);
	if (css_enabled==1)
		server.printP(Page_start_css);
	server.printP(Page_start2);

  // check for parameters
  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, NAMELEN, value, VALUELEN);
      if (rc != URLPARAM_EOS) {
        params_present=true;
        
		#ifdef DEBUG
		// debug output for parameters
		server.print(name);
        server.printP(Parsed_item_separator);
        server.print(value);
        server.printP(Tail_end);
		#endif
				
		param_number=atoi(name);
 		
		// Analog output type and Digital I/O type
		if (param_number >=0xC0 && param_number <=0xC9) 
		{
		  byte out3 = atoi(value);
		  EEPROM.put( param_number, out3 );
        }
		
		// Analog Output Value in %
		else if (param_number ==0xCA || param_number ==0xCB) 
		{
          float out3 = atof(value);
			analog_out[param_number-0xCA]=out3;
        }
		
		
		// Digital Out Value
		if (param_number >=0xD0 && param_number <=0xD7) 
		{
            if (screen_type[(param_number-0xD0)+26]>10) //A pulse counter
			{
				float out3 = atoi(value);
				EEPROM.put( 0x300+4*(param_number-0xD0), out3);
			}
			else
			{
				byte out3 = atoi(value);
				bitWrite(io_digital, param_number-0xD0, out3);
			}
			
        }
		
		// Digital Out Value
		if (param_number >=0xE0 && param_number <=0xE7) 
		{
				float out3 = atof(value);
				EEPROM.put( 0xD0+4*(param_number-0xE0), out3);
			
        }
		
		load_eeprom();
      }
    }
  }
  	
	server.printP(IO_setup_head);
  		 
	 server.printP(table_start);
	 
	 server.printP(table_tr_start);
	 server.printP(table_th_start);
	 server.printP(Analog_out_text);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_cal_cal);
	 server.printP(Analog_cal_offset);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_cal_cal);
	 server.printP(Analog_cal_rampa);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_out_type);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_out_valor);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_save);
	 server.printP(table_th_end);
	 server.printP(table_tr_end);
	 
	 byte l=0;
	 
	 while (l<2)
	 {
		 byte btemp1;
		 float offset=0,rampa=0;
		 
		 if (screen_type[24+l]==1)
		 {
			 offset=anout_offsetV[l];
			 rampa=anout_slopeV[l];
		 }
		 else if (screen_type[24+l]==2)
		 {
			 offset=anout_offsetI[l];
			 rampa=anout_slopeI[l];
		 }
		 
		 server.printP(table_tr_start);
		 server.printP(Form_io_start1);
		server.print(l+1);
		server.printP(Form_io_start2);
		server.printP(table_td_start);
		server.print(l+1);
		server.printP(table_td_end);
		
		server.printP(table_td_start);
		server.print(offset);
		server.printP(table_td_end);
		server.printP(table_td_start);
		server.print(rampa);
		server.printP(table_td_end);
		
		 server.printP(table_td_start);
		 EEPROM.get( 0xC0+l, btemp1 );
		 drop_form_builder(server, strings_anout,192+l, 3, btemp1+1);
		 server.printP(table_td_end);
		
		 server.printP(table_td_start);
		server.printP(Form_input_text_start);
		server.print(0xCA+l);
		server.printP(Form_input_value);
		server.print(analog_out[l],2);
		server.printP(Form_input_size5);
		server.printP(Form_input_end_norm);
		server.printP(table_td_end);
		server.printP(table_td_start);
		server.printP(Form_input_send);
		server.printP(table_td_end);
		server.printP(table_tr_end);
		server.printP(Form_end);
	
		 
		 l++;
	 }
	 
	 server.printP(table_end);
	
	server.printP(IO_digital_head);
	
	 server.printP(table_start);
	 
	 server.printP(table_tr_start);
	 server.printP(table_th_start);
	 server.printP(Digital_text);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_out_type);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_out_valor);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_save);
	 server.printP(table_th_end);
	 server.printP(table_tr_end);
	 
	 l=0;
	 
	 while (l<8)
	 {
		 byte btemp1;
		 server.printP(table_tr_start);
		 server.printP(Form_io_start1);
		server.print(l+3);
		server.printP(Form_io_start2);
		server.printP(table_td_start);
		server.print(l+1);
		server.printP(table_td_end);
		server.printP(table_td_start);
		btemp1=io_type[l+2];
		if (btemp1>=35)
			btemp1=btemp1-32;
		drop_form_builder(server, strings_dionumeros, strings_diotipos,0xC2+l, 7, btemp1+1);
		btemp1=io_type[l+2];
		server.printP(table_td_end);
		 server.printP(table_td_start);
		 if (btemp1==2) //salida
		 {
			server.printP(Form_input_text_start);
			server.print(0xD0+l);
			server.printP(Form_input_value);
			btemp1=bitRead(io_digital, l);
			server.print(btemp1);
			server.printP(Form_input_size2);
			server.printP(Form_input_end_norm); 
		 }
		 else if (btemp1==1) //Entrada
		 {
			server.print(Indio.digitalRead(l+1));
			server.print(" - ");
			server.print(pulse_counter[l]);
		 }
		 else if (btemp1>=10) //Entrada pulsos
		 {
			server.printP(Form_input_text_start);
			server.print(0xD0+l);
			server.printP(Form_input_value);
			server.print(reed_calib[l]);
			server.printP(Form_input_size5);
			server.printP(Form_input_end_norm);
			server.print(" - ");
			server.print(pulse_counter[l]);
		 }
		server.printP(table_td_end);
		server.printP(table_td_start);
		server.printP(Form_input_send);
		server.printP(table_td_end);
		server.printP(table_tr_end);
		server.printP(Form_end);
	
		 
		 l++;
	 }
	 
	 server.printP(table_end);
 	
	server.printP(Page_end);

}


void setupNetwork(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  URLPARAM_RESULT rc;
  char name[NAMELEN];
  char value[VALUELEN];
  boolean params_present = false;
  byte param_number = 0;

  /* this line sends the standard "we're all OK" headers back to the
     browser */
  server.httpSuccess();

  /* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
  if (type == WebServer::HEAD)
    return;

	server.printP(Page_start10);
	server.print(Serial);
	server.printP(Page_start11);
	if (css_enabled==1)
		server.printP(Page_start_css);
	server.printP(Page_start2);

  // check for parameters
  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, NAMELEN, value, VALUELEN);
      if (rc != URLPARAM_EOS) {
        params_present=true;
        
		#ifdef DEBUG
		// debug output for parameters
		server.print(name);
        server.printP(Parsed_item_separator);
        server.print(value);
        server.printP(Tail_end);
		#endif
		
		param_number=atoi(name);
 		
		// read Analog Type values
		if (param_number >=0x10 && param_number <=0x33) 
		{
          byte out3 = atoi(value);
			#ifdef DEBUG
			// debug output for parameters
			server.print(param_number);
			server.printP(Parsed_item_separator);
			server.print(out3);
			server.printP(Tail_end);
			#endif
		  EEPROM.put( param_number, out3 );
        }
		
		if (param_number == 80) 
		{
          int out3 = atoi(value);
			#ifdef DEBUG
			// debug output for parameters
			server.print(param_number);
			server.printP(Parsed_item_separator);
			server.print(out3);
			server.printP(Tail_end);
			#endif
		  EEPROM.put( 0x2C, out3 );
        }
		
		if (param_number == 90) 
		{
		    byte i=0;
			while (i<15)
				{
					EEPROM.put( 0x330+i, value[i]);
					i++;
				}
			EEPROM.put( 0x330+i, 0);
        }
		
		if (param_number == 100) 
		{
          byte out3 = atoi(value);
		  EEPROM.put( 0x190, out3 );
        }
		
		if (param_number == 101) 
		{
          byte out3 = atoi(value);
		  EEPROM.put( 0x191, out3 );
        }
		
		if (param_number == 140) 
		{
		    byte i=0;
			while (i<32)
				{
					EEPROM.put( 0x340+i, value[i]);
					i++;
				}
			EEPROM.put( 0x340+i, 0);
        }
		
		if (param_number == 150) 
		{
		    byte i=0;
			while (i<32)
				{
					EEPROM.put( 0x380+i, value[i]);
					i++;
				}
			EEPROM.put( 0x380+i, 0);
        }
		
		if (param_number == 160) 
		{
		    byte i=0;
			while (i<32)
				{
					EEPROM.put( 0x3C0+i, value[i]);
					i++;
				}
			EEPROM.put( 0x3C0+i, 0);
        }
		
		load_eeprom();
      }
    }
  }
  
    server.printP(Network_setup_head);
  	
	server.printP(Target_IP);
	server.printP(Form_network_start1);
	server.print(150);
	server.printP(Form_network_start2);
	server.printP(Form_input_text_start);
	server.print(150);
	server.printP(Form_input_value);
	EEPROM.get( 0x380, target_adrs );
	server.print(target_adrs);
	server.printP(Form_input_size32);
	server.printP(Form_input_end_norm);
	
	server.printP(Form_input_send);
	server.printP(Form_end);		
	
	byte btemp1;
	
	server.printP(Net_type_txt);
	server.printP(Form_network_start1);
	server.print(101);
	server.printP(Form_network_start2);
	EEPROM.get( 0x191, btemp1 );
	drop_form_builder(server, net_type_drop,101, 2, btemp1+1);
	server.printP(Form_input_send);
	server.printP(Form_end);	
	
	server.printP(Gateway_IP);
	server.printP(Form_network_start1);
	server.print(15);
	server.printP(Form_network_start2);
	
	byte i=0;
	while (i<4)
	{	
		server.printP(Form_input_text_start);
		server.print(0x30+i);
		server.printP(Form_input_value);
		EEPROM.get( 0x30+i, btemp1 );
		server.print(btemp1);
		server.printP(Form_input_size3);
		server.printP(Form_input_end_norm);
		i++;
	}
	server.printP(Form_input_send);
	server.printP(Form_end);	
	
	int itemp1;	
	server.printP(Target_PORT);
	server.printP(Form_network_start1);
	server.print(80);
	server.printP(Form_network_start2);
	server.printP(Form_input_text_start);
	server.print(80);
	server.printP(Form_input_value);
	EEPROM.get( 0x2C, itemp1 );
	server.print(itemp1);
	server.printP(Form_input_size5);
	server.printP(Form_input_end_norm);
	
	server.printP(Form_input_send);
	server.printP(Form_end);	
	
	server.printP(Packet_type);
	server.printP(Form_network_start1);
	server.print(100);
	server.printP(Form_network_start2);
	EEPROM.get( 0x190, btemp1 );
	drop_form_builder(server, strings_packet_type,100, 3, btemp1+1);
	server.printP(Form_input_send);
	server.printP(Form_end);	
	
	server.printP(Target_AKEY);
	server.printP(Form_network_start1);
	server.print(2);
	server.printP(Form_network_start2);
	server.printP(Form_input_text_start);
	server.print(0x28);
	server.printP(Form_input_value);
	EEPROM.get( 0x28, btemp1 );
	server.print(btemp1);
	server.printP(Form_input_size3);
	server.printP(Form_input_end_norm);
	
	server.printP(Form_input_send);
	server.printP(Form_end);	
	
	server.printP(Target_EMON_prefix);
	server.printP(Form_network_start1);
	server.print(160);
	server.printP(Form_network_start2);
	server.printP(Form_input_text_start);
	server.print(160);
	server.printP(Form_input_value);
	EEPROM.get( 0x3C0, EMON_prefix );
	server.print(EMON_prefix);
	server.printP(Form_input_size32);
	server.printP(Form_input_end_norm);
	
	server.printP(Form_input_send);
	server.printP(Form_end);
	
	server.printP(Target_APIKEY);
	server.printP(Form_network_start1);
	server.print(140);
	server.printP(Form_network_start2);
	server.printP(Form_input_text_start);
	server.print(140);
	server.printP(Form_input_value);
	EEPROM.get( 0x340, APIKEY );
	server.print(APIKEY);
	server.printP(Form_input_size32);
	server.printP(Form_input_end_norm);
	
	server.printP(Form_input_send);
	server.printP(Form_end);
	
	server.printP(Target_NODE);
	server.printP(Form_network_start1);
	server.print(90);
	server.printP(Form_network_start2);
	server.printP(Form_input_text_start);
	server.print(90);
	server.printP(Form_input_value);
	EEPROM.get( 0x330, NODE );
	server.print(NODE);
	server.printP(Form_input_size10);
	server.printP(Form_input_end_norm);
	
	server.printP(Form_input_send);
	server.printP(Form_end);	
	
	server.printP(Network_Announce_time);
	server.printP(Form_network_start1);
	server.print(4);
	server.printP(Form_network_start2);
	server.printP(Form_input_text_start);
	server.print(0x2A);
	server.printP(Form_input_value);
	EEPROM.get( 0x2A, btemp1 );
	server.print(btemp1);
	server.printP(Form_input_size3);
	server.printP(Form_input_end_norm);
	
	server.printP(Form_input_send);
	server.printP(Form_end);	
	
	server.printP(Page_end);

}

void setupModbus(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  URLPARAM_RESULT rc;
  char name[NAMELEN];
  char value[VALUELEN];
  boolean params_present = false;
  byte param_number = 0;

  /* this line sends the standard "we're all OK" headers back to the
     browser */
  server.httpSuccess();

  /* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
  if (type == WebServer::HEAD)
    return;

	server.printP(Page_start10);
	server.print(Serial);
	server.printP(Page_start11);
	if (css_enabled==1)
		server.printP(Page_start_css);
	server.printP(Page_start2);

  // check for parameters
  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, NAMELEN, value, VALUELEN);
      if (rc != URLPARAM_EOS) {
        params_present=true;
        
		#ifdef DEBUG
		// debug output for parameters
		server.print(name);
        server.printP(Parsed_item_separator);
        server.print(value);
        server.printP(Tail_end);
		#endif
		
		param_number=atoi(name);
 		
		// read Modbus enable params
		if (param_number >=0x10 && param_number <=0x17) 
		{
            byte out3 = atoi(value);
			#ifdef DEBUG
			// debug output for parameters
			server.print(param_number);
			server.printP(Parsed_item_separator);
			server.print(out3);
			server.printP(Tail_end);
			#endif
		    bitWrite(modbus_enabled[0], param_number-0x10,out3 );
		    EEPROM.put( 0x40, modbus_enabled[0] );
        }
		if (param_number >=0x18 && param_number <=0x1F) 
		{
            byte out3 = atoi(value);
			#ifdef DEBUG
			// debug output for parameters
			server.print(param_number);
			server.printP(Parsed_item_separator);
			server.print(out3);
			server.printP(Tail_end);
			#endif
			bitWrite(modbus_enabled[1], param_number-0x18,out3 );
		    EEPROM.put( 0x41, modbus_enabled[1] );
        }
		
		load_eeprom();
		
      }
	  
    }
  }
  
    server.printP(Modbus_head);
  	
	 server.printP(table_start);
	 
	 server.printP(table_tr_start);
	 server.printP(table_th_start);
	 server.printP(Modbus_Aparato);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Modbus_ID);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Modbus_con_stat);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Modbus_req);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Modbus_suc_req);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Modbus_total_err);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Modbus_valor);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Modbus_habilitar);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Modbus_guardar);
	 server.printP(table_th_end);
	 server.printP(table_tr_end);
	 byte l=0;
	 
	 while (l<12)
	 {
		 byte btemp1;
		 server.printP(table_tr_start);
		 server.printP(Form_modbus_start1);
		server.print(l);
		server.printP(Form_modbus_start2);
		 server.printP(table_td_start);
		 if (l==0)
			server.printP(Modbus_Eastron);
		 else if (l==1)
			server.printP(Modbus_Eastron);
		 else if (l==2)
			server.printP(Modbus_TUF);
		 else if (l==3)
			server.printP(Modbus_Overdigit);
		 else if (l<8)
			server.printP(Modbus_TUF);	
		 else if (l<12)
			server.printP(Modbus_Temp);		 
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 server.print(packets[l].id);
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 server.print(packets[l].connection);
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 server.print(packets[l].requests);
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 server.print(packets[l].successful_requests);
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 server.print(packets[l].total_errors);
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 if (l<4)
			server.print("N/A");
		 else if (l<8)
			server.print(TUF_flow[l-4]);
		 else if (l<12)
		 {
			server.print(temperaturas[(l-8)*2]); server.print(" ");	server.print(temperaturas[1+(l-8)*2]);
		 }
		 
		 
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 server.printP(Form_input_text_start);
		 if (l<8)
		 {
			server.print(0x10+l);
			 server.printP(Form_input_value);
			 btemp1=bitRead(modbus_enabled[0], l );
			server.print(btemp1); 
		 }
		 else
		 {
			server.print(0x10+l);
			 server.printP(Form_input_value);
			 btemp1=bitRead(modbus_enabled[1], l-8 );
			server.print(btemp1); 			 
		 }
		 
		server.printP(Form_input_size2);
		server.printP(Form_input_end_norm);
		server.printP(table_td_end);
		server.printP(table_td_start);
		server.printP(Form_input_send);
		server.printP(table_td_end);
		 server.printP(table_tr_end);
		 server.printP(Form_end);
		 
		 l++;
	 }
	 
	 
	 server.printP(table_end);
	
	server.printP(Page_end);

}


void setupRemote(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  URLPARAM_RESULT rc;
  char name[NAMELEN];
  char value[VALUELEN];
  boolean params_present = false;
  byte param_number = 0;

  /* this line sends the standard "we're all OK" headers back to the
     browser */
  server.httpSuccess();

  /* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
  if (type == WebServer::HEAD)
    return;

	server.printP(Page_start10);
	server.print(Serial);
	server.printP(Page_start11);
	if (css_enabled==1)
		server.printP(Page_start_css);
	server.printP(Page_start2);

  // check for parameters
  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, NAMELEN, value, VALUELEN);
      if (rc != URLPARAM_EOS) {
        params_present=true;
        
		#ifdef DEBUG
		// debug output for parameters
		server.print(name);
        server.printP(Parsed_item_separator);
        server.print(value);
        server.printP(Tail_end);
		#endif
		
		param_number=atoi(name);
		
		load_eeprom();
		
      }
	  
    }
  }
  
    server.printP(Remote_head);
  	
	 server.printP(table_start);
	 
	 server.printP(table_tr_start);
	 server.printP(table_th_start);
	 server.printP(Remoto_numero);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Remoto_tipo);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Remoto_rf);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Remoto_cont);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Remoto_tiempo);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Remoto_valor);
	 server.printP(table_th_end);
	 server.printP(table_tr_end);
	 
	 byte l=0;
	 
	 time_temp=millis();
	 while (l<20)
	 {
		 if (screen_type[l+58]>0)
		 {
			 server.printP(table_tr_start);
			 server.printP(table_td_start);
			 server.print(l);
			 server.printP(table_td_end);
			 
			 server.printP(table_td_start);
			 server.print(myStrings[screen_type[l+58]]);
			 server.printP(table_td_end);
			 
			 server.printP(table_td_start);
			 server.print(remote_rf[l]);
			 server.printP(table_td_end);
			 
			 server.printP(table_td_start);
			 server.print(remote_packets[l]);
			 server.printP(table_td_end);
			 
			 server.printP(table_td_start);
			 time_temp=(time_temp-remote_last[l])/1000;
			 server.print(time_temp);
			 server.printP(table_td_end);
			 
			 server.printP(table_td_start);
			 server.print(remote_val[l]);
			 server.printP(table_td_end);
			 
			 server.printP(table_tr_end);
		 }
		 l++;
	 }
	
	 
	 
	 server.printP(table_end);
	
	server.printP(Page_end);

}

void setupAnalog(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  AlarmConf AlarmTemp;
  
  URLPARAM_RESULT rc;
  char name[NAMELEN];
  char value[VALUELEN];
  boolean params_present = false;
  byte param_number = 0;

  /* this line sends the standard "we're all OK" headers back to the
     browser */
  server.httpSuccess();

  /* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
  if (type == WebServer::HEAD)
    return;

	server.printP(Page_start10);
	server.print(Serial);
	server.printP(Page_start11);
	if (css_enabled==1)
		server.printP(Page_start_css);
	server.printP(Page_start2);

  // check for parameters
  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, NAMELEN, value, VALUELEN);
      if (rc != URLPARAM_EOS) {
        params_present=true;
        
		#ifdef DEBUG
		// debug output for parameters
		server.print(name);
        server.printP(Parsed_item_separator);
        server.print(value);
        server.printP(Tail_end);
		#endif
		
        param_number = atoi(name);
 
        // read Analog Calib values
        if (param_number >=10 && param_number <=25) 
		{
          float out2 = atof(value);
		  EEPROM.put( 0x50+((param_number-10)*4), out2 );
        }
		
		// read Analog Type values
		if (param_number >=26 && param_number <=29) 
		{
          byte out3 = atoi(value);
			#ifdef DEBUG
			// debug output for parameters
			server.print(param_number);
			server.printP(Parsed_item_separator);
			server.print(out3);
			server.printP(Tail_end);
			#endif
		  EEPROM.put( 0x90+(param_number-26), out3 );
        }
				
		// read Analog Type values
		if (param_number >=30 && param_number <45) 	//First
			EEPROM.get( 0x200, AlarmTemp);
		else if (param_number >=45 && param_number <60) 	//First
			EEPROM.get( 0x220, AlarmTemp);
		else if (param_number >=60 && param_number <75) 	//First
			EEPROM.get( 0x240, AlarmTemp);
		else if (param_number >=75 && param_number <90) 	//First
			EEPROM.get( 0x260, AlarmTemp);
         
		
		if (param_number >=30 && param_number <90)
		{
			byte temporal1=param_number-30;
			temporal1 = temporal1 % 15;
		
			switch (temporal1) {
			case 0:
				AlarmTemp.habilitado = atoi(value);
				break;
			case 1:
				AlarmTemp.mayor = atoi(value);
				break;
			case 2:
				AlarmTemp.latching = atoi(value);
				break;
			case 3:
				AlarmTemp.esperar = atoi(value);
				break;
			case 4:
				AlarmTemp.auto_unlatch = atoi(value);
				break;
			case 5:
				AlarmTemp.nivelON = atof(value);
				break;
			case 6:
				AlarmTemp.delayON = atof(value);
				break;
			case 7:
				AlarmTemp.nivelOFF = atof(value);
				break;
			case 8:
				AlarmTemp.delayOFF = atof(value);
				break;
			case 9:
				AlarmTemp.TminOFF = atof(value);
				break;
			}
		}
		
      
		// read Analog Type values
		if (param_number >=30 && param_number <45) 	//First
			EEPROM.put( 0x200, AlarmTemp);
		else if (param_number >=45 && param_number <60) 	//First
			EEPROM.put( 0x220, AlarmTemp);
		else if (param_number >=60 && param_number <75) 	//First
			EEPROM.put( 0x240, AlarmTemp);
		else if (param_number >=75 && param_number <90) 	//First
			EEPROM.put( 0x260, AlarmTemp);
        
		
		load_eeprom();
		
		 
		 
		 
     
      }
    }
  }
  
	float ain_temp[4];
	 float ain_corr[4];
	 
	 ain_temp[0]=Indio.analogRead(1); //Read Analog-In (output depending on selected mode)
	 ain_temp[1]=Indio.analogRead(2); //Read Analog-In (output depending on selected mode)
	 ain_temp[2]=Indio.analogRead(3); //Read Analog-In (output depending on selected mode)
	 ain_temp[3]=Indio.analogRead(4); //Read Analog-In (output depending on selected mode)
	 
	 byte l=0;
	 while (l<4)
	 {
		ain_temp[l]=(ain_temp[l]-offset_to_ma)*slope_to_ma;
		ain_corr[l]=(ain_temp[l]-offset[l])*slope[l];
		l++;
	 }
	 
	 server.printP(Analog_head);
	 
	 
	 server.printP(table_start);
	 
	 server.printP(table_tr_start);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_number);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_ma);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_corrected);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_type);
	 server.printP(table_th_end);
	 server.printP(table_tr_end);
	 l=0;
	 
	 while (l<4)
	 {
		 byte btemp1;
		 
		 server.printP(table_tr_start);
		 server.printP(table_td_start);
		 server.print(l+1);
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 
		 if ((screen_type[l+20]>=35)&&(screen_type[l+20]<=38)) //Opto
		 {
			 server.print("Opto");
			 server.printP(table_td_end);
			 server.printP(table_td_start);
			 server.print(Q_flow_opto[l]);
		 }
		 else
		 {
			 server.print(ain_temp[l],3);
			 server.printP(table_td_end);
			 server.printP(table_td_start);
			 server.print(ain_corr[l],3);
		 }
		 
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 EEPROM.get( 0x90+l, btemp1 );
		 server.print(btemp1);
		 server.printP(table_td_end);
		 
		 l++;
	 }
	 
	 server.printP(table_end);
	 
	 server.printP(Analog_head_calib);
	 
	 l=0;
	 server.print(l); server.print(": "); server.print(myStrings[l]); 
	 while (l<14)
	 {
		 l++;
		 server.print(", "); server.print(l); server.print(": "); server.print(myStrings[l]); 
	 }
	 
	 l=0;
	 while (l<4)
	 {
		 server.print(", "); server.print(l+35); server.print(": "); server.printP(Analog_pulsos); server.print(myStrings[4+l]);  
		 l++;
	 }
	 server.print("<br>\n");
	 
	 server.printP(table_start);
	 server.printP(table_tr_start);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_number);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_X1);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_Y1);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_X2);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_Y2);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_type);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_save);
	 server.printP(table_th_end);
	 server.printP(table_tr_end);
	 
	 l=0;
	 while (l<4)
	 {
		float ftemp1;
		byte btemp1;
				
		
		server.printP(table_tr_start);
		server.printP(Form_analog_start1);
		server.print(l+1);
		server.printP(Form_analog_start2);
		server.printP(table_td_start);
		server.print(l+1);
		server.printP(table_td_end);
		
		byte j=0;
		while(j<4)
		{		
			server.printP(table_td_start);
			server.printP(Form_input_text_start);
			server.print(10+j+(4*l));
			server.printP(Form_input_value);
			EEPROM.get( 0x50+(j*4)+(16*l), ftemp1 );
			server.print(ftemp1,4);
			server.printP(Form_input_size10);
			server.printP(Form_input_end_norm);
			server.printP(table_td_end);
			j++;
		}
		
		
		server.printP(table_td_start);
		server.printP(Form_input_text_start);
		server.print(26+l);
		server.printP(Form_input_value);
		EEPROM.get( 0x90+l, btemp1 );
		server.print(btemp1);
		server.printP(Form_input_size3);
		server.printP(Form_input_end_norm);
		server.printP(table_td_end);
		server.printP(table_td_start);
		server.printP(Form_input_send);
		server.printP(table_td_end);
		server.printP(table_tr_end);
		server.printP(Form_end);
		
		l++;
		
	 }
	 
	 server.printP(table_end);
	 
	 server.printP(Analog_alarm_head);
	 server.printP(table_start);
	 server.printP(table_tr_start);
	 server.printP(table_th_start);	server.printP(Analog_alarm_01); server.printP(table_th_end);
	 server.printP(table_th_start);	server.printP(Analog_alarm_02); server.printP(table_th_end);
	 server.printP(table_th_start);	server.printP(Analog_alarm_03); server.printP(table_th_end);
	 server.printP(table_th_start);	server.printP(Analog_alarm_04); server.printP(table_th_end);
	 server.printP(table_th_start);	server.printP(Analog_alarm_05); server.printP(table_th_end);
	 server.printP(table_th_start);	server.printP(Analog_alarm_06); server.printP(table_th_end);
	 server.printP(table_th_start);	server.printP(Analog_alarm_07); server.printP(table_th_end);
	 server.printP(table_th_start);	server.printP(Analog_alarm_08); server.printP(table_th_end);
	 server.printP(table_th_start);	server.printP(Analog_alarm_09); server.printP(table_th_end);
	 server.printP(table_th_start);	server.printP(Analog_alarm_10); server.printP(table_th_end);
	 server.printP(table_th_start);	server.printP(Analog_alarm_11); server.printP(table_th_end);
	 server.printP(table_th_start); server.printP(Analog_in_text_save); server.printP(table_th_end);
	 server.printP(table_tr_end);
	 
	 l=0;
	 while (l<4)
	 {
		AlarmConf AlarmTemp;
		EEPROM.get( 0x200+32*l, AlarmTemp);		
		
		server.printP(table_tr_start);
		server.printP(Form_analog_start1);
		server.print(l+5);
		server.printP(Form_analog_start2);
		server.printP(table_td_start);
		server.print(l+1);
		server.printP(table_td_end);
		
		byte ll=0;
		
		while (ll<10)
		{
		server.printP(table_td_start);
		server.printP(Form_input_text_start);
		server.print(ll+30+l*15);
		server.printP(Form_input_value);
		switch (ll) {
		case 0:
			server.print(AlarmTemp.habilitado);
			break;
		case 1:
			server.print(AlarmTemp.mayor);
			break;
		case 2:
			server.print(AlarmTemp.latching);
			break;
		case 3:
			server.print(AlarmTemp.esperar);
			break;
		case 4:
			server.print(AlarmTemp.auto_unlatch);
			break;
		case 5:
			server.print(AlarmTemp.nivelON);
			break;
		case 6:
			server.print(AlarmTemp.delayON);
			break;
		case 7:
			server.print(AlarmTemp.nivelOFF);
			break;
		case 8:
			server.print(AlarmTemp.delayOFF);
			break;
		case 9:
			server.print(AlarmTemp.TminOFF);
			break;
		}
		
		server.printP(Form_input_size5);
		server.printP(Form_input_end_norm);
		server.printP(table_td_end);
		
		ll++;
		}
		
		server.printP(table_td_start);
		server.printP(Form_input_send);
		server.printP(table_td_end);
		server.printP(table_tr_end);
		server.printP(Form_end);
		
		l++;
		
	 }
  
	 server.printP(table_end);
  server.printP(Page_end);

}

void setupADAM(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  URLPARAM_RESULT rc;
  char name[NAMELEN];
  char value[VALUELEN];
  boolean params_present = false;
  int param_number = 0;

  /* this line sends the standard "we're all OK" headers back to the
     browser */
  server.httpSuccess();

  /* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
  if (type == WebServer::HEAD)
    return;

	server.printP(Page_start10);
	server.print(Serial);
	server.printP(Page_start11);
	if (css_enabled==1)
		server.printP(Page_start_css);
	server.printP(Page_start2);

  // check for parameters
  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, NAMELEN, value, VALUELEN);
      if (rc != URLPARAM_EOS) {
        params_present=true;
        
		#ifdef DEBUG
		// debug output for parameters
		server.print(name);
        server.printP(Parsed_item_separator);
        server.print(value);
        server.printP(Tail_end);
		#endif
		
        param_number = atoi(name);
 
        // read Analog Calib values
        if (param_number >=10 && param_number <=31) 
		{
          float out2 = atof(value);
		  EEPROM.put( 0x100+((param_number-10)*4), out2 );
        }
		
		// read Analog Type values
		if (param_number >=50 && param_number <=58) 
		{
          byte out3 = atoi(value);
			#ifdef DEBUG
			// debug output for parameters
			server.print(param_number);
			server.printP(Parsed_item_separator);
			server.print(out3);
			server.printP(Tail_end);
			#endif
		  EEPROM.put( 0x180+(param_number-50), out3 );
        }
		
		if (param_number == 9) 
		{
          byte out3 = atoi(value);
			#ifdef DEBUG
			// debug output for parameters
			server.print(param_number);
			server.printP(Parsed_item_separator);
			server.print(out3);
			server.printP(Tail_end);
			#endif
		  EEPROM.put( 0xF0, out3 );
        }
		
		load_eeprom();
		
		 
		 
		 
     
      }
    }
  }
	  
	 
	 server.printP(adam_head);
	 
	 byte btemp2;
	 
	server.printP(adam_enabled_text);
	server.printP(Form_adam_start1);
	server.print(40);
	server.printP(Form_adam_start2);
	server.printP(Form_input_text_start);
	server.print(9);
	server.printP(Form_input_value);
	EEPROM.get( 0xF0, btemp2 );
	server.print(btemp2);
	server.printP(Form_input_size3);
	server.printP(Form_input_end_norm);
	server.printP(Form_input_send);
	server.printP(Form_end);
	 
	 server.printP(table_start);
	 
	 server.printP(table_tr_start);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_number);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_ma);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_corrected);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_type);
	 server.printP(table_th_end);
	 server.printP(table_tr_end);
	 
	 byte l=0;
	 
	 while (l<8)
	 {
		 byte btemp1;
		 server.printP(table_tr_start);
		 server.printP(table_td_start);
		 server.print(l+1);
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 server.print(adam_read[l],3);
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 server.print(ain_values_corrected[l+4],3);
		 server.printP(table_td_end);
		 server.printP(table_td_start);
		 EEPROM.get( 0x180+l, btemp1 );
		 server.print(btemp1);
		 server.printP(table_td_end);
		 		 
		 l++;
	 }
	 
	 server.printP(table_end);
	 
	 server.printP(Analog_head_calib);
	 
	 l=0;
	 server.print(l); server.print(": "); server.print(myStrings[l]); 
	 while (l<14)
	 {
		 l++;
		 server.print(", "); server.print(l); server.print(": "); server.print(myStrings[l]); 
	 }
	 server.print("<br>\n");
	 
	 server.printP(table_start);
	 server.printP(table_tr_start);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_number);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_X1);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_Y1);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_X2);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_Y2);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_type);
	 server.printP(table_th_end);
	 server.printP(table_th_start);
	 server.printP(Analog_in_text_save);
	 server.printP(table_th_end);
	 server.printP(table_tr_end);
	 
	 l=0;
	 while (l<8)
	 {
		float ftemp1;
		byte btemp1;
				
		
		server.printP(table_tr_start);
		server.printP(Form_adam_start1);
		server.print(l+1);
		server.printP(Form_adam_start2);
		server.printP(table_td_start);
		server.print(l+1);
		server.printP(table_td_end);
		
		byte j=0;
		while(j<4)
		{		
			server.printP(table_td_start);
			server.printP(Form_input_text_start);
			server.print(10+j+(4*l));
			server.printP(Form_input_value);
			EEPROM.get( 0x100+(j*4)+(16*l), ftemp1 );
			server.print(ftemp1,4);
			server.printP(Form_input_size10);
			server.printP(Form_input_end_norm);
			server.printP(table_td_end);
			j++;
		}
		
		
		server.printP(table_td_start);
		server.printP(Form_input_text_start);
		server.print(50+l);
		server.printP(Form_input_value);
		EEPROM.get( 0x180+l, btemp1 );
		server.print(btemp1);
		server.printP(Form_input_size3);
		server.printP(Form_input_end_norm);
		server.printP(table_td_end);
		server.printP(table_td_start);
		server.printP(Form_input_send);
		server.printP(table_td_end);
		server.printP(table_tr_end);
		server.printP(Form_end);
		
		l++;
		
	 }
	 
	 server.printP(table_end);
	 	 
	 server.printP(Form_end);
	 
  
  server.printP(Page_end);

}

void logInfo(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  URLPARAM_RESULT rc;
  char name[NAMELEN];
  char value[VALUELEN];
  boolean params_present = false;
  byte param_number = 0;

  /* this line sends the standard "we're all OK" headers back to the
     browser */
  
    // check for parameters
  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, NAMELEN, value, VALUELEN);
      if (rc != URLPARAM_EOS) {
        params_present=true;
        
		/*
		#ifdef DEBUG
		// debug output for parameters
		server.print(name);
        server.printP(Parsed_item_separator);
        server.print(value);
        server.printP(Tail_end);
		#endif
		*/
		
		if (strcmp(name, "download") == 0) 
		{
		    server.httpSuccess("text/plain");
			uint8_t buffersd[32];
			
			File dataFile = SD.open(value);
			
			  // if the file is available, write to it:
			  if (dataFile) {
				while (dataFile.available()) {
				  if (dataFile.available()>32)
				  {
					dataFile.read(&buffersd,32);
					server.write(buffersd,32);
				  }
				  else{
					server.write(dataFile.read());  
				  }
				  
				  Watchdog.reset();	//Resetear el Watchdog
				}
				dataFile.close();
			  }
			  // if the file isn't open, pop up an error:
			  else {
				server.printP(Log_sd_error);
				server.print(value);
			  }
			return;
        }
		
		if (strcmp(name, "dellog") == 0) 
		{
		    SD.remove("log.txt");
        }
		
		param_number=atoi(name);
 		
      }
    }
  }
  
  server.httpSuccess();
  
  
  
  /* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
  if (type == WebServer::HEAD)
    return;

	server.printP(Page_start10);
	server.print(Serial);
	server.printP(Page_start11);
	if (css_enabled==1)
		server.printP(Page_start_css);
	server.printP(Page_start2);

  // check for parameters
  if (strlen(url_tail)) {
    while (strlen(url_tail)) {
      rc = server.nextURLparam(&url_tail, name, NAMELEN, value, VALUELEN);
      if (rc != URLPARAM_EOS) {
        params_present=true;
        
		#ifdef DEBUG
		// debug output for parameters
		server.print(name);
        server.printP(Parsed_item_separator);
        server.print(value);
        server.printP(Tail_end);
		#endif
		
		if (param_number == 140) 
		{
		    byte i=0;
			while (i<32)
				{
					EEPROM.put( 0x340+i, value[i]);
					i++;
				}
			EEPROM.put( 0x340+i, 0);
        }
		
		load_eeprom();
      }
    }
  }
  
    server.printP(Log_head);
	
	server.printP(Log_reinicios);
    server.print(reboot_count);
	server.printP(Line_break);
	server.printP(Log_del_sistema);
	server.printP(Log_scroll);
	
	File dataFile = SD.open("log.txt");
	byte cartemp;
	// if the file is available, write to it:
	if (dataFile) {
		while (dataFile.available()) {
		  cartemp=dataFile.read();
		  if (cartemp=='\n')
			server.print("<br>");
		  else
			server.write(cartemp);
		  Watchdog.reset();	//Resetear el Watchdog
		}
		dataFile.close();
	}
	// if the file isn't open, pop up an error:
	else {
		server.printP(Log_sd_error);
		server.print("log.txt");
	}
	server.printP(Log_div_end);
	server.printP(Log_sd);
	
	root = SD.open("/");	
	printDirectory(server, root, 0, 1);
	
	server.printP(Page_end);

}


void errorHTML(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete)
{
  /* this line sends the standard "HTTP 400 Bad Request" headers back to the
     browser */
  server.httpFail();

  /* if we're handling a GET or POST, we can output our data here.
     For a HEAD request, we just stop after outputting headers. */
  if (type == WebServer::HEAD)
    return;
    
  server.printP(Http400);
  
  server.printP(Page_end);
}

void draw_screen()
{
  u8g.firstPage();  //Dibujar la pantalla
  do {
    draw();
  } while( u8g.nextPage() );
}

void setup()
{
  EEPROM.start();
    
  load_eeprom();	//Load Analog Inputs calibration values to offset[4] and slope[4] and others
  
  // Reset ethernet module
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(RESET_PIN, LOW);
  delayMicroseconds(50);
  digitalWrite(RESET_PIN, HIGH);
  // Wait for the module to reset
  
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(10, OUTPUT);
  
  digitalWrite(4, HIGH);
  digitalWrite(6, HIGH);
  digitalWrite(10, HIGH);
  
  SPI.begin();
  //SPI.setDataMode(SPI_MODE0);  
  //SPI.setBitOrder(MSBFIRST);
  //SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  //digitalWrite(10, LOW);
  
  pinMode(backlightPin, OUTPUT); //set backlight pin to output 
  
  byte e0;
  byte e1;
  EEPROM.get( 0x00, e0 );
  EEPROM.get( 0x01, e1 );
  
  if ((digitalRead(25)==0&&digitalRead(23)==0)||(('O'!=e0)&&('K'!=e1)))
  {
	burnDefaults();
	byte out2;
	out2 = 'O';	EEPROM.put( 0x00, out2 );
	out2 = 'K';	EEPROM.put( 0x01, out2 );
	for (int ck=0;ck<20;ck++){
		analogWrite(backlightPin, 0); //set backligth to MAX
		delay(100);
		analogWrite(backlightPin, 255); //set backligth to MAX
		delay(50);
	}
	
  }
  
  
  countdownMS = Watchdog.enable(20000); //Enable Watchdog 
  
  RTCind.get(rtc,true); 
  RTCind.start(true);
  
  u8g.setRot180();	//Rotate Screen
  analogWrite(backlightPin, 255); //set backligth to MAX
 
  u8g.firstPage();  //Dibujar el Logo
  do {
     print_intro_logo();
  } while( u8g.nextPage() );
  delay(2000);
  Watchdog.reset();	//Resetear el Watchdog
    
  while (init_status<10)
  {  
	  u8g.firstPage();  //Dibujar
	  do {
		inicializacion(); //Definir el dibujo
	  } while( u8g.nextPage() );
	  
	  switch (init_status)
	  {
		  case 0:
			delay (500);	//Espera qa que se muestren los datos del sistema
			Watchdog.reset();	//Resetear el Watchdog
		  case 1:
			//Ethernet.begin(mac, ip, dnServer ,gateway,subnet);
			//Ethernet.begin(mac);
			if (net_type==0) //DHCP enabled
			{
				if (Ethernet.begin(mac) == 0){ //Try DHCP and if it fails try loading fixed IP
					Ethernet.begin(mac, ip, dnServer ,gateway,subnet);
					ethOK=1; //Static IP
					}
				else{
					ethOK=2; //DHCP
				}
			}
			else{
				Ethernet.begin(mac, ip, dnServer ,gateway,subnet);
				ethOK=1; //Static IP
			}
			
			delay(1000);
			w5500.setRetransmissionTime(10000); //1 second
			w5500.setRetransmissionCount(2);
			delay(100);
			Watchdog.reset();	//Resetear el Watchdog
			//Arrancar el Ethernet
			break;
		  case 2:
			Udp.begin(8887);	//Arrancar el socket de UDP, para enviar datos y para recibir los sensores externos
			Watchdog.reset();	//Resetear el Watchdog
			break;
		  case 3:	//Arrancar el webserver
			  webserver.setDefaultCommand(&indexHTML);
			  webserver.setFailureCommand(&errorHTML);
			  webserver.addCommand("index.html", &indexHTML);
			  webserver.addCommand("analog.html", &setupAnalog);
			  webserver.addCommand("io.html", &ioPage);
			  webserver.addCommand("adam.html", &setupADAM);
			  webserver.addCommand("network.html", &setupNetwork);
			  webserver.addCommand("modbus.html", &setupModbus);
			  webserver.addCommand("remote.html", &setupRemote);
			  webserver.addCommand("log.html", &logInfo);
			  
			  webserver.begin();
			  Watchdog.reset();	//Resetear el Watchdog
			break;
		  case 4:
				Watchdog.reset();	//Resetear el Watchdog
				if (syncron_enable==0)
				{
					//Sync here wathever you want to sync at boot
					SyncOK=2;
				}
				else if (clientk.connect(target_adrs, 80))
				{
					SyncOK=1;
				}
				clientk.flush();
				clientk.stop();
				Watchdog.reset();	//Resetear el Watchdog
			break;
		  case 5:
				delay(500);
				if (!card.init(SPI_HALF_SPEED, SD_CS1)||!SD.begin(SD_CS1))
					SDOK=0;
				else
					SDOK=1;
			break;
		  case 6:
			break;
				
	  }
	  
	  init_status++;
  }

  Watchdog.reset();	//Resetear el Watchdog
  
  load_fram();
  
  for (int i=0; i < 8; i++){
		byte btemp1=io_type[i+2];
		if (btemp1==0) //Deshabilitado
		{
			pulse_counter[i]=0;
			u_2.ul=pulse_counter[i];
			FRAMWrite(0x10+4*i, u_2.asbyte, 4);
		}
	}
  
  //attachInterrupt(7, count, CHANGE); //Si se realiza una llama a Indio.read dentro de la interrupción peta.
  
  
  delay(2000);
  

  Indio.digitalWrite(1,LOW);	//Reset relay output
  Indio.digitalMode(1,OUTPUT);  // Set CH1 as an output (Alarm rela)
  Indio.digitalMode(2,INPUT);  // Set CH2 as an input (Alarm reset button)|
    

  
  
  
  //Set Modbus Packets for EASTRON SDM630M-CT
  // read 18 registers (9 values)starting at address 0x00
  packet1->id = 1;
  packet1->function = 4;	//Read function
  packet1->address = 0x00;
  packet1->no_of_registers = 18;
  packet1->register_array = regs;
  
  // read 6 registers (3 values)starting at address 0x1E 
  packet2->id = 1;
  packet2->function = 4;	//Read function
  packet2->address = 0x1E;
  packet2->no_of_registers = 6;
  packet2->register_array = regs2;
  
  // read 2 registers (1 value) starting at address 0x01 in the TUF-2000 flowmeter
  packet3->id = 2;
  packet3->function = 3;	//Read function
  packet3->address = 0x00;
  packet3->no_of_registers = 2;
  packet3->register_array = regs3;
  
  // read 8 registers (8 values) starting at address 0x00 in the Overdigit
  packet4->id = 4;
  packet4->function = 4;	//Read function
  packet4->address = 0x00;
  packet4->no_of_registers = 8;
  packet4->register_array = regs4;
  
  // read 2 registers (1 value) starting at address 0x01 in the TUF-2000 flowmeter
  packet5->id = 10;
  packet5->function = 3;	//Read function
  packet5->address = 0x00;
  packet5->no_of_registers = 2;
  packet5->register_array = regs5;
  
  // read 2 registers (1 value) starting at address 0x01 in the TUF-2000 flowmeter
  packet6->id = 11;
  packet6->function = 3;	//Read function
  packet6->address = 0x00;
  packet6->no_of_registers = 2;
  packet6->register_array = regs6;
  
  // read 2 registers (1 value) starting at address 0x01 in the TUF-2000 flowmeter
  packet7->id = 12;
  packet7->function = 3;	//Read function
  packet7->address = 0x00;
  packet7->no_of_registers = 2;
  packet7->register_array = regs7;
  
  // read 2 registers (1 value) starting at address 0x01 in the TUF-2000 flowmeter
  packet8->id = 13;
  packet8->function = 3;	//Read function
  packet8->address = 0x00;
  packet8->no_of_registers = 2;
  packet8->register_array = regs8;
  
  // read 2 registers 0x1000 in the Omega temp controller
  packet9->id = 20;
  packet9->function = 3;	//Read function
  packet9->address = 0x1000;
  packet9->no_of_registers = 2;
  packet9->register_array = regs9;
  
    // read 2 registers 0x1000 in the Omega temp controller
  packet10->id = 21;
  packet10->function = 3;	//Read function
  packet10->address = 0x1000;
  packet10->no_of_registers = 2;
  packet10->register_array = regs10;
  
    // read 2 registers 0x1000 in the Omega temp controller
  packet11->id = 22;
  packet11->function = 3;	//Read function
  packet11->address = 0x1000;
  packet11->no_of_registers = 2;
  packet11->register_array = regs11;
  
    // read 2 registers 0x1000 in the Omega temp controller
  packet12->id = 23;
  packet12->function = 3;	//Read function
  packet12->address = 0x1000;
  packet12->no_of_registers = 2;
  packet12->register_array = regs12;
  
  
     
  // Initialize communication settings etc...
  modbus_configure(baud, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS);
  
  byte k1=0;
  while (k1<8)
  {
	 packets[k1].connection=bitRead(modbus_enabled[0], k1); 
	 k1++;
  }
  k1=0;
  while (k1<4)
  {
	 packets[k1+8].connection=bitRead(modbus_enabled[1], k1);
	 k1++;
  }
    
  Indio.setADCResolution(16); // Set the ADC resolution. Choices are 12bit@240SPS, 14bit@60SPS, 16bit@15SPS and 18bit@3.75SPS.
  Indio.analogReadMode(1, mA_raw);	//Set Inputs to 4-20mA RAW
  Indio.analogReadMode(2, mA_raw);
  Indio.analogReadMode(3, mA_raw);
  Indio.analogReadMode(4, mA_raw);
  Indio.digitalWrite(8,LOW);
  Indio.digitalMode(8,OUTPUT);  // Set CH8 as an output (Purge relay output)
    
  time1=millis();
  purge_time=86400000;
  String print3;
  
  
  File myFile = SD.open("log.txt", FILE_WRITE);
  myFile.print("[");
  myFile.print(reboot_count);
  myFile.print(" - ");
  RTCind.get(rtc,true);
  print3=print3+rtc[6]+"/"+rtc[5]+"/"+rtc[4]+" "+rtc[2]+":"+rtc[1]+":"+rtc[0];
  myFile.print(print3);
  myFile.print("]");
  myFile.println(" Boot OK");
  // close the file:
  myFile.close();
}

//Rutina de interrupción, que se llama cada vez que cambia un pin digital. Peligros: El sei() no debería de utilizarse, pero de no utilizarlo el SPI no funciona, y por tanto el Indio.digitalRead se cuelga.
// No se debería cambiar variables globales... y si se utilizan hay que utilizar volatile: http://arduino.stackexchange.com/questions/20994/why-the-need-to-use-the-volatile-keyword-on-global-variables-when-handling-inter
//More info http://www.gammon.com.au/interrupts
void count(){
	//sei();//dangerous!!!!
	for (int i=0; i < 8; i++){
		byte btemp1=io_type[i+2];
		if (btemp1==1||btemp1>10)
		{
			//Is input or counter
			if (0==bitRead(dig_inputs, i)&&Indio.digitalRead(i+1)==1) //Rising edge
			{
				pulse_counter[i]++;
			}
			bitWrite(dig_inputs,i,Indio.digitalRead(i+1));
		}
	}
}

byte read_analog_buttons()
{
  if ((digitalRead(25)==1)&&(digitalRead(24)==1)&&(digitalRead(23)==1)) //No key pressed
  {
	  last_cycle_key=0;
	  return 0;
  }
  
  if ((digitalRead(25)==0) && (last_cycle_key==0))	//UP
  {
	  delay(20);	//Debounce
	  last_key_press=millis();
	  last_cycle_key=3;
	  return 3; 
	  
  }
  else if ((digitalRead(24)==0) && (last_cycle_key==0)) //SELECT
  {
	  delay(20);
	  last_key_press=millis();
	  last_cycle_key=1;
	  return 1;
		  
  }
  else if ((digitalRead(23)==0) && (last_cycle_key==0)) //DOWN
  {
	  delay(20);
	  last_key_press=millis(); 
	  last_cycle_key=2;
	  return 2;
  }
  else
  {
	  return 0;  // default
  }
}

void next_FSM()
{
	byte k3=0; //loop exit flag
	byte k4=0; //loop overrun counter
	
	adc_key = read_analog_buttons();
	
	if (FSM_status!=0)	//Menu watchdog
	{
		time_temp3=millis();
		if (time_temp-last_key_press>=100000)
			FSM_status=0;
	}
		
	switch (FSM_status) {
    
	case 0:    // Main Menu
      switch (adc_key) {
	  case 1:	//Enter pressed
		FSM_status=0;
		break;
	  case 2:	//DOWN pressed
		screen_number--;
		
		k3=0; //loop exit flag
		k4=0; //loop overrun counter
		while (k3==0)
		{
			if (screen_number>127)
				screen_number=127;
			
			if (screen_type[screen_number]==255||screen_type[screen_number]==0)
					screen_number--;
				else
					k3=1;
			
			k4++;
			
			
			
			if (k4>127)		//Bucle por si se si no hay ningun Ain activador
				{
					k4=0;
					k3=1;
					screen_number=0;
				}
		}
		last_screen_change=millis();
		break;
		
	case 3:	//UP pressed
		screen_number++;
		
		k3=0; //loop exit flag
		k4=0; //loop overrun counter
		while (k3==0)
		{
			if (screen_number>127)
				screen_number=0;
			
			if (screen_type[screen_number]==255||screen_type[screen_number]==0)
					screen_number++;
				else
					k3=1;
			
			k4++;
			
			if (screen_number>127)
				screen_number=0;
			
			if (k4>127)		//Bucle por si se si no hay ningun Ain activador
				{
					k4=0;
					k3=1;
					screen_number=0;
				}
		}
		last_screen_change=millis();
		
		break;
	  }		
		
	  //cambio automatico del screen
	  time_temp3=millis();
	  if ((time_temp3-last_key_press>=30000)&&(time_temp3-last_screen_change>=5000))
	  {
		screen_number++;
		
		k3=0; //loop exit flag
		k4=0; //loop overrun counter
		while (k3==0)
		{
			if (screen_number>127)
				screen_number=0;
			
			if (screen_type[screen_number]==255||screen_type[screen_number]==0)
					screen_number++;
				else
					k3=1;
			
			k4++;
			
			if (k4>127)		//Bucle por si se si no hay ningun Ain activador
				{
					k4=0;
					k3=1;
					screen_number=0;
				}
		}
		last_screen_change=millis();
	    }
		break;
	  }
		
	}

void load_eeprom()
{
	byte j=0;
	
	while (j<4)
	{
		udp_ip[j]=EEPROM.read(0x20+j);
		j++;
	}
	
	j=0;
	while (j<4)
	{
		gateway[j]=EEPROM.read(0x30+j);
		j++;
	}
	
	
	
	
	Akey=EEPROM.read(0x28);
	Node=EEPROM.read(0x29);
	
	syncron_enable=EEPROM.read(0x2B);
	
	send_packet_type=EEPROM.read(0x190);
	net_type=EEPROM.read(0x191);
	
	EEPROM.get( 0x2C, PORT_UDP );
	
	EEPROM.get( 0x340, APIKEY );
	
	EEPROM.get( 0x380, target_adrs );
	
	EEPROM.get( 0x3C0, EMON_prefix );
	
	EEPROM.get( 0x320, SERIALN );
	EEPROM.get( 0x330, NODE );

  float t1, t2, t3, t4;
  
  j=0;
	while (j<2)
	{
		EEPROM.get( 0xD0+16*j, t1 );
		EEPROM.get( 0xD4+16*j, t2 );
		EEPROM.get( 0xD8+16*j, t3 );
		EEPROM.get( 0xDC+16*j, t4 );
		anout_offsetV[j]=t1;
		anout_slopeV[j]=t2;
		anout_offsetI[j]=t3;
		anout_slopeI[j]=t4;
		j++;
	}
  
  byte k=0,type_tmp;
  while (k<4)							//Load Analog In calib values and calculate Slope and Offset
  {
	  EEPROM.get( 0x50+16*k, t1 );
	  EEPROM.get( 0x54+16*k, t2 );
	  EEPROM.get( 0x58+16*k, t3 );
	  EEPROM.get( 0x5C+16*k, t4 );
	  slope[k]=(t4-t2)/(t3-t1);
	  offset[k]=t1-t2/slope[k];
	  EEPROM.get( 0x90+k, type_tmp );
	  ain_type[k]=type_tmp;
	  screen_type[k+20]=type_tmp;
	  k++;
	  
  }
  
  k=0;	//Load ADAM-4017 values
  while (k<8)
  {
	  EEPROM.get( 0x100+16*k, t1 );
	  EEPROM.get( 0x104+16*k, t2 );
	  EEPROM.get( 0x108+16*k, t3 );
	  EEPROM.get( 0x10C+16*k, t4 );
	  slope[k+4]=(t4-t2)/(t3-t1);
	  offset[k+4]=t1-t2/slope[k+4];
	  EEPROM.get( 0x180+k, type_tmp );
	  ain_type[4+k]=type_tmp;
	  screen_type[k+40]=type_tmp;
	  k++;
	  
  }
  
  //Load Adam-4017 Enable parameter
  EEPROM.get( 0xF0, adam_enabled );
  
  //Load Web posting Interval
  EEPROM.get( 0x2A, type_tmp );
  #ifndef DEBUG
	if (type_tmp<5)
		type_tmp=type_tmp+5;
  #endif
  post_interval=(type_tmp)*1000;
    
  //Load I/O
  j=0;
  while (j<10) //Load I/O
	{
		EEPROM.get( 0xC0+j, type_tmp);	
		io_type[j]=type_tmp;
		screen_type[j+24]=type_tmp;
		j++;
	}
  
  
  j=0;
	AlarmConf AlarmTemp;
	
	while (j<4) //Load alarms
	{
		EEPROM.get( 0x200+32*j, AlarmTemp);	
		Alarmas[j]=AlarmTemp;
		j++;
	}
	
	j=0;
	while (j<8) //Load alarms
	{
		EEPROM.get( 0x300+4*j, t1);	
		reed_calib[j]=t1;
		j++;
	}	
	
	EEPROM.get( 0x40, modbus_enabled[0]);
	EEPROM.get( 0x41, modbus_enabled[1]);	
    
	j=0;
	while (j<4)
	{
		screen_type[50+j]=bitRead(modbus_enabled[0],4+j);
		screen_type[54+j]=bitRead(modbus_enabled[1],j);
		j++;
	}
  
}

void load_fram()
{
	FRAMRead(1, u_2.asbyte, 4);
	delay(10);
	reboot_count=u_2.ul;
	reboot_count++;
	u_2.ul=reboot_count;
	delay(10);
	FRAMWrite(1, u_2.asbyte, 4); 
	
	for (int i=0; i < 8; i++){
		FRAMRead(0x10+4*i, u_2.asbyte, 4);
		pulse_counter[i]=u_2.ul;
	}
}

void burnDefaults()
{
	byte out2;
	
		out2 = 0;	//Burn ADAM-4017 Enable to 0
		EEPROM.put( 0xF0, out2 );
	
		out2 = 8;
		EEPROM.put( 0x20, out2 );
		out2 = 8;
		EEPROM.put( 0x21, out2 );
		out2 = 8;
		EEPROM.put( 0x22, out2 );
		out2 = 8;
		EEPROM.put( 0x23, out2 );
		out2 = 192;
		EEPROM.put( 0x30, out2 );
		out2 = 168;
		EEPROM.put( 0x31, out2 );
		out2 = 1;
		EEPROM.put( 0x32, out2 );
		out2 = 1;
		EEPROM.put( 0x33, out2 );
		out2 = 1;
		EEPROM.put( 0x28, out2 );
		out2 = 1;
		EEPROM.put( 0x29, out2 );
		
		out2 = 10;
		EEPROM.put( 0x2A, out2 );
		
		out2 = 1;
		EEPROM.put( 0x2B, out2 );
		
		out2 = 0;
		EEPROM.put( 0xA0, out2 );
		out2 = 2;
		EEPROM.put( 0xA1, out2 );
		out2 = 5;
		EEPROM.put( 0xA2, out2 );
		out2 = 1;
		EEPROM.put( 0xAE, out2 );
			
		out2 = 2;
		EEPROM.put( 0x190, out2 ); //Send_packet_type
		
		out2 = 0;
		EEPROM.put( 0x191, out2 ); //Send_packet_type
		
		int out3 = 80;
		EEPROM.put( 0x2C, out3 );
	
		char char1[]="R/W API Key";
		EEPROM.put( 0x340, char1 );
		char char2[]="www.emoncms.org";
		EEPROM.put( 0x380, char2 );
		char char3[]="";
		EEPROM.put( 0x3C0, char3 );
		
		out3 = 100;
		EEPROM.put( 0xA8, out3 );
		
		float out4 = 11;
		EEPROM.put( 0xAA, out4 );
		
		out4 = 1000;
		EEPROM.put( 0xB0, out4 );
		out3 = 1000;
		EEPROM.put( 0xA8, out3 );
		
		float ftemp1;
		byte w=0;
		while (w<4)
		{
						
			ftemp1=4;
			EEPROM.put( 0x50+16*w, ftemp1 );
			ftemp1=4;
			EEPROM.put( 0x54+16*w, ftemp1 );
			ftemp1=20;
			EEPROM.put( 0x58+16*w, ftemp1 );
			ftemp1=20;
			EEPROM.put( 0x5C+16*w, ftemp1 );
			out2=9;
			EEPROM.put( 0x90+w, out2 );
			w++;
		}
		
		 byte j=0;
		while (j<2)
		{
			ftemp1=0;
			EEPROM.put( 0xD0+16*j, ftemp1 );
			EEPROM.put( 0xD8+16*j, ftemp1 );
			ftemp1=1;
			EEPROM.put( 0xD4+16*j, ftemp1 );
			EEPROM.put( 0xDC+16*j, ftemp1 );
			j++;
		}
		
		w=0;
		while (w<8)
		{
						
			ftemp1=4;
			EEPROM.put( 0x100+16*w, ftemp1 );
			ftemp1=4;
			EEPROM.put( 0x104+16*w, ftemp1 );
			ftemp1=20;
			EEPROM.put( 0x108+16*w, ftemp1 );
			ftemp1=20;
			EEPROM.put( 0x10C+16*w, ftemp1 );
			out2=0;
			EEPROM.put( 0x180+w, out2 );
			w++;
		}
		
		AlarmConf AlarmTemp;
		
		j=0;
		AlarmTemp.habilitado=0;
		AlarmTemp.mayor=0;
		AlarmTemp.latching=1;
		AlarmTemp.esperar=0;
		AlarmTemp.auto_unlatch=0;
		AlarmTemp.nivelON=0;
		AlarmTemp.delayON=0;
		AlarmTemp.nivelOFF=1;
		AlarmTemp.delayOFF=0;
		AlarmTemp.TminOFF=0;
		
		while (j<4) //Load alarms
		{
			EEPROM.put( 0x200+32*j, AlarmTemp);	
			j++;
		}
		
		out4 = 1000;
		j=0;
		while (j<8) //Burn flowmter
		{
			EEPROM.put( 0x300+4*j, out4);	
			j++;
		}	
		
		//I/O Types
		j=0;
		out2=0;
		while (j<10) //Load I/O
		{
			EEPROM.put( 0xC0+j, out2);	
			j++;
		}
		
		j=255;
		EEPROM.put( 0x40, j);
		j=0;
		EEPROM.put( 0x41, j);
		
	//FRAM
	u_2.ul=0;  
	FRAMWrite(1, u_2.asbyte, 4);
}

void ReadAin()
{
	if (ain_type[ain_read]!=0)
	{
		ain_values_corrected[ain_read]=(Indio.analogRead(ain_read+1)-offset_to_ma)*slope_to_ma;
		ain_values_corrected[ain_read]=(ain_values_corrected[ain_read]-offset[ain_read])*slope[ain_read];
	}
	else
		ain_values_corrected[ain_read]=0;
	ain_read++;
	unsigned int connection_status = modbus_update(packets);
	if (ain_read==4)
		ain_read=0;
}

void ModbusToFloat()					//Convert modbus registers to float
{
	byte w=0;
	while (w<9){						//Eastron V,I,P
		u.asfloat[1]=regs[w*2];
		u.asfloat[0]=regs[w*2+1];
		eastron_read[w]=u.f;
		#ifdef EASTRON_WATT
		if (w>5)
			eastron_read[w]=eastron_read[w]*0.001;
		#endif
		w++;
	}
	
	w=0;
	while (w<3){						//Eastron P.F.
		u.asfloat[1]=regs2[w*2];
		u.asfloat[0]=regs2[w*2+1];
		eastron_read[w+9]=u.f;
		w++;
	}
	
	w=0;
	while (w<8){						//ADAM-4017 read
		int tint1;
		tint1=regs4[w];
		adam_read[w]=tint1;
		adam_read[w]=(adam_read[w]-offset_overdigit_ma)*slope_overdigit_ma;
		ain_values_corrected[w+4]=(adam_read[w]-offset[w+4])*slope[w+4];
		w++;
	}
	
	u.asfloat[1]=regs3[1];				//Flowmeter read
	u.asfloat[0]=regs3[0];
	Q_flow_485=(u.f);
	
	u.asfloat[1]=regs5[1];				//Flowmeter read
	u.asfloat[0]=regs5[0];
	TUF_flow[0]=(u.f);
	u.asfloat[1]=regs6[1];				//Flowmeter read
	u.asfloat[0]=regs6[0];
	TUF_flow[1]=(u.f);
	u.asfloat[1]=regs7[1];				//Flowmeter read
	u.asfloat[0]=regs7[0];
	TUF_flow[2]=(u.f);
	u.asfloat[1]=regs8[1];				//Flowmeter read
	u.asfloat[0]=regs8[0];
	TUF_flow[3]=(u.f);
	
	
	
	float ftemp1;
	ftemp1 = regs9[0];
	temperaturas[0]=ftemp1*0.1;
	ftemp1 = regs9[1];
	temperaturas[1]=ftemp1*0.1;
	ftemp1 = regs10[0];
	temperaturas[2]=ftemp1*0.1;
	ftemp1 = regs10[1];
	temperaturas[3]=ftemp1*0.1;
	ftemp1 = regs11[0];
	temperaturas[4]=ftemp1*0.1;
	ftemp1 = regs11[1];
	temperaturas[5]=ftemp1*0.1;
	ftemp1 = regs12[0];
	temperaturas[6]=ftemp1*0.1;
	ftemp1 = regs12[1];
	temperaturas[7]=ftemp1*0.1;
}

float ReadPulseInput(byte pulse_pin, byte loop_time, int max_counts, float units_per_pulse)
{
		
	Indio.setADCResolution(12); // Set the ADC resolution. Choices are 12bit@240SPS, 14bit@60SPS, 16bit@15SPS and 18bit@3.75SPS.
	
	Indio.analogReadMode(1, mA_raw);
	  Indio.analogReadMode(2, mA_raw);
	  Indio.analogReadMode(3, mA_raw);
	  Indio.analogReadMode(4, mA_raw);
	
	float cuentas_seg=0,liters_per_second=0;

	byte ligth=0;
	int threshold=0;
	byte uptemp=0;
	byte downtemp=0;

	byte last=0;
	int contador=0;
	unsigned long int_time=loop_time*1000;
	unsigned long time1,first_up,last_up;
	byte first_up_cycle=0;
	
	time1 = millis();
	
	while ((millis()-time1)<int_time)
	{
		
		Watchdog.reset();
		
			 
		lastSampleI = sampleI;
		sampleI = Indio.analogRead(pulse_pin);
		lastFilteredI = filteredI;
		filteredI = 0.996*(lastFilteredI+sampleI-lastSampleI);    
	   if (ligth==0 && filteredI>threshold)
	   {
		 if (uptemp<2)
		   uptemp=uptemp+1;
		 else
		 {
		   ligth=1;
		   uptemp=0;
		   downtemp=0;
		 }
	   }
	   
	   if (ligth==1 && filteredI<threshold)
	   {
		 if (downtemp<2)
		   downtemp=downtemp+1;
		 else
		 {
		   ligth=0;
		   uptemp=0;
		   downtemp=0;
		 }
	   }
	  
	  
	  if (last==0 && ligth==1)
	  {
		if (first_up_cycle==0)
		{
			first_up_cycle=1;
			first_up = millis();
		}
		else
		{
			last_up = millis();
		}
		contador=contador+1;
	  }
	  last=ligth;
	  if (contador>max_counts)
		  break;
	   
	}
	
	if (contador>2)
	{
		cuentas_seg=contador;
		cuentas_seg=cuentas_seg*1000;
		cuentas_seg=cuentas_seg/(last_up-first_up);
	}
	else
		cuentas_seg=0;
	
	Indio.setADCResolution(16); // Set the ADC resolution. Choices are 12bit@240SPS, 14bit@60SPS, 16bit@15SPS and 18bit@3.75SPS.
	
	Indio.analogReadMode(1, mA_raw);
	Indio.analogReadMode(2, mA_raw);
	Indio.analogReadMode(3, mA_raw);
	Indio.analogReadMode(4, mA_raw);
	
	liters_per_second=cuentas_seg*units_per_pulse;
		
	return liters_per_second;
}

float read_reed_input(byte channel)
{
	float reed_flow=Q_flow_reed[channel];
	float total_power=eastron_read[6]+eastron_read[7]+eastron_read[8];
	#ifdef EASTRON_WATT
		total_power=total_power*0.001;
	#endif
	if ((total_power<5)&&screen_type[26+channel]>=100)
	{
		reed_counter[channel]=0;
		reed_flow=0;
	}
	
	if ((Indio.digitalRead(channel+1)==1)&&last_reed[channel]==0)	//Rising edge detection
	{
		
		last_reed[channel]=1;
		time_temp=millis();
		
		reed_flow=(time_temp-time_reed_last[channel])*0.001;
		reed_flow=reed_constant[channel]/reed_flow;
		
		time_reed_last[channel]=time_temp;
		
		float ftemp2=1+reed_counter[channel];
		ftemp2=1/ftemp2;
		
		reed_flow=((Q_flow_reed[channel]*reed_counter[channel])+reed_flow)*ftemp2;

		if (reed_counter[channel]<5)
			reed_counter[channel]++;
	}
	
	if (Indio.digitalRead(channel+1)==0)
	{
		last_reed[channel]=0;
	}
		
	return reed_flow;
}

void post_UDP()
{
	display_bottom_status=1;
	draw_screen();
	
	Udp.beginPacket(target_adrs, PORT_UDP);
    Udp.print(Akey);
	if (send_packet_type == 1)
		{
			Udp.print(" ");
			Udp.print(SERIALN);
		}	Udp.print(" ");
	Udp.print(NODE);
	
	byte w=0;
	while (w<9){	//Mandar parametros del Analizador
		Udp.print(" ");
		if (eastron_read[w]==0){
			Udp.print("0");
		}else{
			Udp.print(eastron_read[w],1);
		}
		w++;
	}
	while (w<12){	//Mandar parametros del Analizador
		Udp.print(" ");
		if (eastron_read[w]==0){
			Udp.print("0");
		}else{
			Udp.print(eastron_read[w],2);
		}
		w++;
	}
	
	w=0;
	while (w<4){
		byte type_temp=screen_type[w+20];
		Udp.print(" ");
		
		if (send_packet_type == 1)
		{
			Udp.print(type_temp);
			Udp.print(":");
		}
		
		if (((type_temp>=35)&&(type_temp<=38))||((type_temp>=135)&&(type_temp<=138))){
			if (Q_flow_opto[w]==0){
				Udp.print("0");
			}else{
				Udp.print(Q_flow_opto[w],2);
			}
		}else{
			if (ain_values_corrected[w]==0){
				Udp.print("0");
			}else{
				Udp.print(ain_values_corrected[w],2);
			}
		}
		w++;
	}
	
	w=0;
	while (w<8)
	{
		byte type_temp=screen_type[26+w];		
		Udp.print(" ");
		
		if (send_packet_type == 1)
		{
			Udp.print(type_temp);
			Udp.print(":");
		}
		
		if (type_temp==1)	//Input
		{
			if (Indio.digitalRead(1+w)==1)
				Udp.print("1");
			else
				Udp.print("0");
		}
		
		else if (type_temp==2)	//Output
		{
			if (1==bitRead(io_digital, w))
				Udp.print("1");
			else
				Udp.print("0");
		}
		else if ((type_temp>=35)&&(type_temp<=38))
		{
			Udp.print(Q_flow_reed[w],2);
		}
		else
		{
			Udp.print("0");
		}
		w++;
	}
	
	w=0;
	while (w<2)
	{
		byte type_temp=screen_type[w+24];	
		
		Udp.print(" ");

		if (send_packet_type == 1)
		{
			Udp.print(type_temp);
			Udp.print(":");
		}
		
		if (type_temp>0)
		{
			if (analog_out[w]==0){
				Udp.print("0");
			}else{
				Udp.print(analog_out[w],2);
			}
		}
		else
			Udp.print("0");
		w++;
	}
	
		
	if (adam_enabled==1)
	{
		
		w=0;
		while (w<8){
			
			byte type_temp=screen_type[w+40];	
			
			Udp.print(" ");
			if (send_packet_type == 1)
			{
				Udp.print(type_temp);
				Udp.print(":");
			}
			
			if (ain_values_corrected[w+4]==0){
				Udp.print("0");
			}else{
				Udp.print(ain_values_corrected[w+4],2);
			}
			w++;
		}
	}
	
	w=0;
	byte send_tuf=0, send_temp=0;
	while (w<4)
	{
		send_tuf=send_tuf + bitRead(modbus_enabled[0], w+4 );
		send_temp=send_temp + bitRead(modbus_enabled[1], w );
		w++;
	}
	
	
	if (send_tuf > 0)
	{
		w=0;
		while (w<4)
		{
			Udp.print(" ");
			if (TUF_flow[w]==0){
				Udp.print("0");
			}else{
				Udp.print(TUF_flow[w],1);
			}
			w++;
		}
	}
	
	if (send_temp > 0)
	{
		w=0;
		while (w<4)
		{
			Udp.print(" ");
			if (temperaturas[w*2]==0){
				Udp.print("0");
			}else{
				Udp.print(temperaturas[w*2],1);
			}
			Udp.print(" ");
			if (temperaturas[w*2+1]==0){
				Udp.print("0");
			}else{
				Udp.print(temperaturas[w*2+1],1);
			}
			w++;
		}
	}
	
	Udp.endPacket();
	display_bottom_status=2;
	draw_screen();
	
}

// this method makes a HTTP connection to the server:
void post_EMON() {
  display_bottom_status=10;
  draw_screen();
  // if there's a successful connection:
  if (clientk.connect(target_adrs, PORT_UDP)) {
	display_bottom_status=11;
	draw_screen();
    // send the HTTP GET request:
    
	
	
	//clientk.print("GET ");
	//clientk.print(target_adrs);
	//clientk.print("/input/post.json?apikey=");
	clientk.print("GET ");
	clientk.print(EMON_prefix);
	clientk.print("/input/post.json?apikey=");
    clientk.print(APIKEY);
      clientk.print("&node=");
      clientk.print(NODE);
    clientk.print("&json={");
	clientk.print(SERIALN);
	clientk.print(":1");
	byte w=0;
	while (w<3){	//Mandar parametros del Analizador
		clientk.print(",");
		clientk.print("V");
		clientk.print(w+1);
		clientk.print(":");
		clientk.print(eastron_read[w],1);
		w++;
	}
	
	w=0;
	while (w<3){	//Mandar parametros del Analizador
		clientk.print(",");
		clientk.print("I");
		clientk.print(w+1);
		clientk.print(":");
		clientk.print(eastron_read[w+3],1);
		
		w++;
	}
	
	w=0;
	while (w<3){	//Mandar parametros del Analizador
		clientk.print(",");
		clientk.print("P");
		clientk.print(w+1);
		clientk.print(":");
		clientk.print(eastron_read[w+6],1);
		
		w++;
	}
	w=0;
	while (w<3){	//Mandar parametros del Analizador
		clientk.print(",");
		clientk.print("PF");
		clientk.print(w+1);
		clientk.print(":");
		clientk.print(eastron_read[w+9],1);
		
		w++;
	}
	
	w=0;
	while (w<4){
		byte type_temp=screen_type[w+20];
		
		clientk.print(",");
		clientk.print("Ain");
		clientk.print(w+1);
		clientk.print("_");
		clientk.print(myStrings[type_temp]);
		clientk.print(":");
		
		if (((type_temp>=35)&&(type_temp<=38))||((type_temp>=135)&&(type_temp<=138))){
				clientk.print(Q_flow_opto[w],2);
		}else{
				clientk.print(ain_values_corrected[w],2);
		}
		
		w++;
	}
	
	w=0;
	while (w<8)
	{
		byte type_temp=screen_type[26+w];		
		clientk.print(",");
		clientk.print("D");
		clientk.print(w+1);
		clientk.print("_");
		if (type_temp<10){
			clientk.print(strings_diotipos[type_temp]);
		}else{
			clientk.print(strings_diotipos[type_temp-32]);
		}
		clientk.print(":");
		if (type_temp==1){	//Input
			if (Indio.digitalRead(1+w)==1)
				clientk.print("1");
			else
				clientk.print("0");
		}else if (type_temp==2){	//Output
			if (1==bitRead(io_digital, w))
				clientk.print("1");
			else
				clientk.print("0");
		}else if ((type_temp>=35)&&(type_temp<=38)){
			clientk.print(Q_flow_reed[w],2);
		}else{
			clientk.print("0");
		}
		w++;
		
	}
	
	w=0;
	while (w<2)
	{
		byte type_temp=screen_type[26+w];		
		clientk.print(",");
		clientk.print("Aout");
		clientk.print(w+1);
		clientk.print("_");
		clientk.print(strings_anout[type_temp]);
		clientk.print(":");
		if (type_temp>0)
		{
			clientk.print(analog_out[w],2);
		}
		else
			clientk.print("0");
		w++;
		
	}
	
	if (adam_enabled==1)
	{
		
		w=0;
		while (w<8){
			
			byte type_temp=screen_type[w+40];
		
			clientk.print(",");
			clientk.print("Ain");
			clientk.print(w+5);
			clientk.print("_");
			clientk.print(myStrings[type_temp]);
			clientk.print(":");
			
			clientk.print(ain_values_corrected[w+4],2);

			
			w++;
			}
	}
	
	w=0;
	byte send_tuf=0, send_temp=0;
	while (w<4)
	{
		send_tuf=send_tuf + bitRead(modbus_enabled[0], w+4 );
		send_temp=send_temp + bitRead(modbus_enabled[1], w );
		w++;
	}
	
	if (send_tuf > 0)
	{
		w=0;
		while (w<4)
		{
			clientk.print(",");
			clientk.print("TUF");
			clientk.print(w+1);
			clientk.print(":");
			clientk.print(TUF_flow[w],1);
			w++;
			
		}
	}
	
	if (send_temp > 0)
	{
		w=0;
		while (w<4)
		{
			clientk.print(",");
			clientk.print("TempPV_");
			clientk.print(w+1);
			clientk.print(":");
			clientk.print(temperaturas[w*2],1);
			clientk.print(",");
			clientk.print("TempSV_");
			clientk.print(w+1);
			clientk.print(":");
			clientk.print(temperaturas[w*2+1],1);
			w++;
			
		}
	}
	
	
	
    clientk.println("} HTTP/1.1");
    clientk.println("Host:emoncms.org");
    clientk.println("User-Agent: Arduino-ethernet");
    clientk.println("Connection: close");
    clientk.println();

    clientk.stop();
	display_bottom_status=12;
	draw_screen();
  }
  else
  {
	display_bottom_status=13;
	draw_screen();
  }
 
}

void print_main()
{
	String title1;
	float valor;
	String title2;
	String print3;
	
	int w;	
	w=screen_number;
	//w=-2; //Temporal force to show EASTRON
	
	
	if (w==0) //Main Screen
	{
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print("Estado");
		u8g.setColorIndex(1);
	}
	else if (w==1) //Networking
	{
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print("Red");
		u8g.setColorIndex(1);
		u8g.setFont(u8g_font_profont11);
		u8g.setPrintPos(1, 24);   	u8g.print("Modo: ");
		byte btemp1;
      EEPROM.get( 0x190, btemp1 );
		u8g.print(strings_packet_type[btemp1]);		
		u8g.setPrintPos(1, 32);   	u8g.print("Akey: "); u8g.print(Akey);
		u8g.setPrintPos(1, 40);   	u8g.print("Nodo: "); u8g.print(NODE);
		u8g.setPrintPos(1, 48);   	u8g.print("IP: "); u8g.print(Ethernet.localIP());
		
	}
	else if (w==2) //Modbus
	{
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print("Modbus");
		u8g.setColorIndex(1);
	}
	else if (w==3) //Remoto
	{
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print("Remoto");
		u8g.setColorIndex(1);
		u8g.setFont(u8g_font_profont11);
		if (last_input_packet_time>0)
		{
			u8g.setPrintPos(1, 24);   	u8g.print(Remote_Line);
			u8g.setPrintPos(1, 32);   	u8g.print("IP:");
			for (int i = 0; i < 4; i++) {
				u8g.print(remote_input_ip[i], DEC);
			  if (i < 3) {
				u8g.print(".");
			  }
			}
			time_temp=millis();
			time_temp=(time_temp-last_input_packet_time)/1000;
			u8g.setPrintPos(1, 40);   	u8g.print("Hace: ");
			u8g.print(time_temp);
		}
		
		
		
	}
	else if (w==4) //Alarm Historic
	{
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print("Alarmas");
		u8g.setColorIndex(1);
	}
	else if (w==5) //RPi
	{
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print("RPi");
		u8g.setColorIndex(1);
	}
	else if (w==6) //Info
	{
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print("Info");
		u8g.setColorIndex(1);
	}
	
	else if (w==7) //RPi Messages
	{
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print("Diagnostico");
		u8g.setColorIndex(1);
	}
	
	else if (w==15) //Analizador de redes
	{
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print("Analizador");
		
		u8g.setColorIndex(1);		
		u8g.setFont(u8g_font_profont11);
		u8g.setPrintPos(1, 24);   	u8g.print("V");
		u8g.setPrintPos(30, 24);   	u8g.print("A");
		u8g.setPrintPos(60, 24);   	u8g.print("kW");
		u8g.setPrintPos(90, 24);   	u8g.print("FP");
		
		u8g.drawLine(2, 25, 126, 25);
		
		byte k5=0,k6=0;
		while (k5<4)
		{
			k6=0;
			while (k6<3)
			{
				u8g.setPrintPos(1+k5*30, 34+k6*9);
				switch (k5)
				{
				case 0:
					u8g.print(eastron_read[k6+k5*3],0);
					break;
				case 1:
					u8g.print(eastron_read[k6+k5*3],0);
					break;
				case 2:
					u8g.print(eastron_read[k6+k5*3],0);
					break;
				case 3:
					u8g.print(eastron_read[k6+k5*3],2);
					break;
				}
				k6++;
			}
			k5++;
		}
	}
	
	else if (w>=20&&w<=23)	//Analog IN
	{
		byte type_temp=screen_type[w];
		if (type_temp>100)
			type_temp=type_temp-100;
		
		if (type_temp>30)
			type_temp=type_temp-30;
		
		
		
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print(w-19); u8g.print("-"); u8g.print(myStrings[type_temp]);
		
		
		
		type_temp=screen_type[w];
		if (type_temp>100)
			type_temp=type_temp-100;
		
		if (type_temp<25)
		{
			valor=ain_values_corrected[w-20];
			u8g.setColorIndex(1);
			u8g.setFont(u8g_font_profont29r);
			u8g.setPrintPos(5, 36);   	u8g.print(valor);
		}
		else if ((type_temp>=35)&&(type_temp<=38))
		{
			valor=Q_flow_opto[w-20];
			u8g.setColorIndex(1);
			u8g.setFont(u8g_font_profont29r);
			u8g.setPrintPos(5, 36);   	u8g.print(valor);
		}
		
		
		u8g.setColorIndex(1);
	}
	
	else if (w>=24&&w<=25)	//A out
	{
		byte type_temp=screen_type[w];		
		
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print(w-23); u8g.print("-"); u8g.print("Aout ");
		
		if (type_temp==1)	//Volt
		{
			u8g.print("Volt");
			u8g.setColorIndex(1);
			u8g.setFont(u8g_font_profont29r);
			u8g.setPrintPos(5, 36);   	u8g.print(analog_out[w-24]);
		}
		
		else if (type_temp==2)	//mA
		{
			u8g.print("mA");
			u8g.setColorIndex(1);
			u8g.setFont(u8g_font_profont29r);
			u8g.setPrintPos(5, 36);		u8g.print(analog_out[w-24]);
		}
		
	}
	
	else if (w>=26&&w<=33)	//Digital I/O
	{
		byte type_temp=screen_type[w];		
		
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print(w-25); u8g.print("-");u8g.print("Dig ");
		
		if (type_temp==1)	//Input
		{
			u8g.print("IN");
			u8g.setColorIndex(1);
			u8g.setFont(u8g_font_profont29r);
			u8g.setPrintPos(5, 36);   	u8g.print(Indio.digitalRead(w-25));
			u8g.setFont(u8g_font_profont11);
			u8g.setPrintPos(1, 53);
			u8g.print(pulse_counter[w-26]);
		}
		
		else if (type_temp==2)	//Output
		{
			u8g.print("OUT");
			u8g.setColorIndex(1);
			u8g.setFont(u8g_font_profont29r);
			u8g.setPrintPos(5, 36);
			if (1==bitRead(io_digital, w-26))
				u8g.print("1");
			else
				u8g.print("0");
		}
		
		else if ((type_temp>=35)&&(type_temp<=38))
		{
			u8g.print("PL");
			u8g.setColorIndex(1);
			u8g.setFont(u8g_font_profont29r);
			u8g.setPrintPos(5, 36);   	u8g.print(Q_flow_reed[w-26]);
			u8g.setFont(u8g_font_profont11);
			u8g.setPrintPos(1, 53);
			u8g.print(Indio.digitalRead(w-25));
			u8g.setPrintPos(32, 53);
			u8g.print(pulse_counter[w-26]);
		}
	}
	
	else if (w>=40&&w<=49)	//Overdigit Extender
	{
		byte type_temp=screen_type[w];
		if (type_temp>100)
			type_temp=type_temp-100;
		
		if (type_temp>30)
			type_temp=type_temp-30;
		
		
		
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print(w-39+4); u8g.print("-"); u8g.print(myStrings[type_temp]);
				
		valor=ain_values_corrected[w-40+4];
		u8g.setColorIndex(1);
		u8g.setFont(u8g_font_profont29r);
		u8g.setPrintPos(5, 36);   	u8g.print(valor);
		
		u8g.setColorIndex(1);
	}
	
	else if (w>=50&&w<=53)	//TUF-200
	{		
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print(w-49); u8g.print("-TUF-2000");
			
			
			u8g.setColorIndex(1);
			u8g.setFont(u8g_font_profont29r);
			u8g.setPrintPos(5, 36);   	u8g.print(TUF_flow[w-50]);
	}
	
	else if (w>=54&&w<=57)	//T_out
	{
		
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print(w-53);u8g.print("-T Ext ");
			
			
			u8g.setColorIndex(1);
			u8g.setFont(u8g_font_profont29r);
			u8g.setPrintPos(5, 36);   	u8g.print(temperaturas[(w-54)*2]);
	}
	
	else if (w>=58&&w<=80)	//Remotos
	{
		byte type_temp=screen_type[w];
		
		u8g.setColorIndex(1);
		u8g.drawBox(0,0,127,16);
		u8g.setColorIndex(0);
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);u8g.print("R"); u8g.print(w-58); u8g.print("-"); u8g.print(myStrings[type_temp]);
		
		
		if (type_temp<25)
		{
			valor=remote_val[w-58];
			u8g.setColorIndex(1);
			u8g.setFont(u8g_font_profont29r);
			u8g.setPrintPos(5, 36);   	u8g.print(valor);
		}

		
		u8g.setColorIndex(1);		
		u8g.setFont(u8g_font_profont11);
		u8g.setPrintPos(1, 52);   	u8g.print("RF: "); u8g.print(remote_rf[w-58]);
		unsigned long time_temp2=millis();
		time_temp2=(time_temp2-remote_last[w-58])/1000;
		u8g.setPrintPos(54, 52);   	u8g.print("Hace: "); u8g.print(time_temp2); u8g.print("s");
		
		
		
		u8g.setColorIndex(1);
	}
	
	
	else 
	{
		u8g.setFont(u8g_font_helvB14);
		u8g.setPrintPos(1, 15);		u8g.print(w);
	}
	

	
	//time_t t = now();
	
	switch (display_bottom_status)
	{
	case 0:
		RTCind.get(rtc,true); 
		print3=print3+rtc[6]+"/"+rtc[5]+"/"+rtc[4]+" "+rtc[2]+":"+rtc[1]+":"+rtc[0];
		//print3=print3+"Ciclos: "+ciclos;
		break;
	case 1:
		print3="UDP Enviando...";
		break;
	case 2:
		print3="UDP Enviado";
		break;
	case 3:
		print3="Contando Pulsos...";
		break;
	case 4:
		print3="Sincronizando hora...";
		break;
	case 10:
		print3="EmonCMS Iniciando...";
		break;
	case 11:
		print3="EmonCMS Enviando...";
		break;
	case 12:
		print3="EmonCMS Enviado";
		break;
	case 13:
		print3="EmonCMS Fail";
		break;
	case 16:
		print3="SD...";
		break;
	case 17:
		print3="SD OK";
		break;	
	}
	
	u8g.setFont(u8g_font_profont11);
	u8g.setPrintPos(1, 64);   	u8g.print(print3);
	
}

void print_intro_logo()
{
  u8g.drawXBMP( 0, 0, Industruino_logo_width, Industruino_logo_height, Industruino_logo_bits);
}

void draw(void) {
	if (FSM_status==0)
		print_main();
}

void inicializacion()
{
	u8g.setFont(u8g_font_unifontr);
	u8g.setPrintPos(0, 11);   u8g.print("System MK1");
	u8g.setFont(u8g_font_profont11);
	if (init_status==0)
	{
		u8g.setPrintPos(0, 20);   	u8g.print("S/N: "); u8g.print(SERIALN);
		u8g.setPrintPos(0, 28);   	u8g.print("S/W: "); u8g.print(sw_ver);
	}
	else{
		u8g.setPrintPos(0, 20);   	u8g.print("Ethernet...");
		if (init_status<2)
			return;
		if (ethOK==0)
			{
				u8g.setPrintPos(105, 20);	u8g.print("FAIL");
			}
		else if (ethOK==1)
			{
				u8g.setPrintPos(92, 20);	u8g.print("STATIC");
			}
		else if (ethOK==2)
			{
				u8g.setPrintPos(105, 20);	u8g.print("DHCP");
			}
		u8g.setPrintPos(0, 28);   	u8g.print("UDP Init...");
		if (init_status<3)
			return;
		u8g.setPrintPos(110, 28);	u8g.print(" OK");
		u8g.setPrintPos(0, 36);   	u8g.print("Webserver Init...");
		if (init_status<4)
			return;
		u8g.setPrintPos(110, 36);	u8g.print(" OK");
		u8g.setPrintPos(0, 44);
		u8g.print(target_adrs);
		u8g.print("...");
			
		
		if (init_status<5)
			return;
		if (SyncOK==0)
			{
				u8g.setPrintPos(105, 44);	u8g.print("FAIL");
			}
		else if (SyncOK==1)
			{
				u8g.setPrintPos(110, 44);	u8g.print(" OK");
			}
		else if (SyncOK==2)
			{
				u8g.setPrintPos(105, 44);	u8g.print("NULL");
			}
		u8g.setPrintPos(0, 52);   	u8g.print("SD Init...");
		if (init_status<6)
			return;
		if (SDOK==0)
			{
				u8g.setPrintPos(105, 52);	u8g.print("FAIL");
			}
		else if (SDOK==1)
			{
				u8g.setPrintPos(110, 52);	u8g.print(" OK");
			}
		if (init_status<7)
			return;
	}
}

void out_logic(byte canal)
{
	byte level_condition, level_OFF_condition, autounlach_condition;
	
	// Si se cumple la regla establecida para el ON
	if (Alarmas[canal].mayor==1)	//Si Mayor que
	{
		if(ain_values_corrected[canal]>Alarmas[canal].nivelON)
		{
			level_condition=1;
			Alarmas_last_on[canal]=millis();
		}
		else
			level_condition=0;
	}
	else	//Si menor que
	{
		if(ain_values_corrected[canal]<Alarmas[canal].nivelON)
		{
			level_condition=1;
			Alarmas_last_on[canal]=millis();
		}
		else
			level_condition=0;
	}
	
	// Si se cumple la regla establecida para el OFF
	if (Alarmas[canal].mayor==1)	//Si Mayor que
	{
		if(ain_values_corrected[canal]<Alarmas[canal].nivelOFF)
		{
			level_OFF_condition=1;
		}
		else
			level_OFF_condition=0;
	}
	else	//Si menor que
	{
		if(ain_values_corrected[canal]>Alarmas[canal].nivelOFF)
		{
			level_OFF_condition=1;
		}
		else
			level_OFF_condition=0;
	}
	
	if (Alarmas[canal].esperar==1&&Alarmas_first_cycle[canal]==0)	//Si esta activado el sistema de espera
	{
		if (level_condition==0)	//Esperar hasta que la condicion no se cumpla
		{
			Alarmas_first_cycle[canal]=1;	//Activar el flag
			Alarmas_last_low_time[canal]=millis();
		}
	}
	else		//Si el sistema de espera está desactivado
		Alarmas_first_cycle[canal]=1;
	
	if (Alarmas_first_cycle[canal]==1)
	{
		if (level_OFF_condition==1)
		{
			time_temp=millis();
			if ((time_temp-Alarmas_last_no_OFF[canal]>=Alarmas[canal].delayOFF*1000)&&(time_temp-Alarmas_last_on[canal]>=Alarmas[canal].TminOFF*1000))
			{
				autounlach_condition=1;
			}
			else
			{
				autounlach_condition=0;
			}
		}
		else
		{
			Alarmas_last_no_OFF[canal]=millis();
			autounlach_condition=0;
		}
		
		if (level_condition==1)
		{
			time_temp=millis();
			if (time_temp-Alarmas_last_low_time[canal]>=Alarmas[canal].delayON*1000)
			{
				Alarmas_out_status[canal]=1;
			}
			else if (Alarmas[canal].latching!=1)
			{
				Alarmas_out_status[canal]=0;
				Alarmas_unlatched_flags[canal]=0;
			}
		}
				
		else
		{
			Alarmas_last_low_time[canal]=millis();
			if (Alarmas[canal].latching==1)
			{
				if (Indio.digitalRead(2)==1)	//if unlatching button pressed
				{
					Alarmas_out_status[canal]=0;
					Alarmas_unlatched_flags[canal]=0;
				}
				

				if (Alarmas[canal].auto_unlatch==1 && autounlach_condition==1)		//If autounlatching is enabled and autounlatch condition satisfies
				{
					Alarmas_out_status[canal]=0;
				}

				
			}
			else
			{
			Alarmas_out_status[canal]=0;
			}
		}
	}
	
	
	if (Alarmas_out_status[canal]==1&&Alarmas[canal].habilitado)
	{
		Alarmas_out_flag[canal]=1;
		Alarmas_unlatched_flags[canal]=1;
	}
	else
	{	
		Alarmas_out_flag[canal]=0;
	}

}

void a_out()
{
	byte l=0;
	while (l<2)
	{
		if (io_type[l]==1)
		{
			Indio.analogWriteMode(l+1, V10);
			Indio.analogWrite(l+1, (analog_out[l]-anout_offsetV[l])*anout_slopeV[l], false);
		}
		else if (io_type[l]==2)
		{
			Indio.analogWriteMode(l+1, mA);
			Indio.analogWrite(l+1, (analog_out[l]-anout_offsetI[l])*anout_slopeI[l], false);
		}
		l++;
	}
	
}

void d_out()
{
	byte l=0;
	while (l<8)
	{
		if (io_type[l+2]==2)
		{
			Indio.digitalMode(l+1,OUTPUT); 
			if (1==bitRead(io_digital, l))
				Indio.digitalWrite(l+1,HIGH);
			else
				Indio.digitalWrite(l+1,LOW);
		}
		else if (io_type[2+l]==1)
		{
			Indio.digitalMode(l+1,INPUT); 
		}
		l++;
	}
}

void eastron_alarm() // Eastron power analyzer alarms
{
	byte k10=0;
	byte alarm_code=0;
	float V_alarm_percent=0.1;
	float I_alarm_percent=0.1;
	float P_alarm_percent=0.1;
	float media_V=0,media_I=0,media_P=0,temp_f1;
	while (k10<3)	//calcular media de V, I, P
	{
		media_V=media_V+eastron_read[k10];
		media_I=media_I+eastron_read[k10+3];
		media_P=media_P+eastron_read[k10+6];
		k10++;
	}
	media_V=media_V/3;
	media_I=media_I/3;
	media_P=media_P/3;
	
	k10=0;
	while (k10<3)
	{
		temp_f1=media_V-eastron_read[k10];
		temp_f1=abs(temp_f1);
		if (temp_f1>(media_V*V_alarm_percent))
		{
			alarm_code=(k10+1);
		}
		k10++;
	}
	
	
}

void alarm_relay_out()
{
	byte k1=0, sum1=0;
  while (k1<4)
  {
	  out_logic(k1);
	  sum1=sum1+Alarmas_out_flag[k1];
	  k1++;
  }
  
    if (Indio.digitalRead(2)==1)	//if unlatching button pressed
	{
		k1=0;
		while (k1<4)
		{
			Alarmas_unlatched_flags[k1]=0;
			k1++;
		}
	}
  
  if (sum1>0)
		Indio.digitalWrite(1,HIGH);
	else
		Indio.digitalWrite(1,LOW); 
}

void purga_compresor()
{
	Indio.digitalWrite(8,HIGH);
	delay(500);
	Indio.digitalWrite(8,LOW);
}

void pulse_meters()
{
	//Read Digital pulse meters 
	byte k=0;
	while (k<8)
	{
		if ((screen_type[26+k]>=35)&&(screen_type[26+k]<=38))
			Indio.digitalMode(k+1,INPUT); 
			Q_flow_reed[k]=read_reed_input(k);
		k++;
	}
}

void parsearUDP()
{
	byte remote_sens=0;
	
	int packetSize = Udp.parsePacket();
	  if (packetSize) {
		remote_input_ip = Udp.remoteIP();
		last_input_packet_time=millis();
		
		byte j=0;
		while (j<INPUT_SIZE+1)
		{
			IncomingUDP[j]=0;
			j++;
		}
		Udp.read(IncomingUDP, INPUT_SIZE);
		IncomingUDP[INPUT_SIZE]=0;
		
		j=0;
		while (j<INPUT_SIZE+1)
		{
			Remote_Line[j]=IncomingUDP[j];
			j++;
		}
	  }
	
	char* command = strtok(IncomingUDP, "&");
	while (command != 0)
	{
		int param_number;
		int par_store;
		
		// Split the command in two values
		char* separator = strchr(command, ':');
		if (separator != 0)
		{
			// Actually split the string in 2: replace ':' with 0
			*separator = 0;
			param_number = atoi(command);
			float fvalue=0;
			int ivalue=0;
			
			if (param_number>=58&&param_number<=80)
			{
				par_store=param_number;
				++separator;
				fvalue = atof(separator);
				remote_sens=1;
				remote_val[param_number-58]=fvalue;
				if (screen_type[param_number]<1)
					screen_type[param_number]=15;
				remote_packets[param_number-58]++;
				remote_last[param_number-58]=millis();
			}
			
			else
			{
				++separator;
				ivalue = atoi(separator);
			}
			
			if (param_number==40&&remote_sens==1)
			{
				screen_type[par_store]=ivalue;
			}
			
			if (param_number==41&&remote_sens==1)
			{
				remote_rf[par_store-58]=ivalue;
			}
			
			
		}
				
		// Find the next command in input string
		command = strtok(0, "&");
	}
	
}

void Post_SD()
{
	display_bottom_status=16;
	draw_screen();
	
	String print3;
	
	/*if (year()<1990)
		print3=String(reboot_count)+".log";
	else{
		time_t t = now();
		print3=String(year(t));
		if (month(t)<10)
			print3=print3+"0";
		print3=print3+month(t);
		if (day(t)<10)
			print3=print3+"0";
		print3=print3+day(t);
		print3=print3+".log";
	}*/
	
	char tab2[13];
	strcpy(tab2, print3.c_str());
	
	File myFile = SD.open(tab2, FILE_WRITE);
	//myFile.print(now());
	myFile.print(" ");	  
	
	byte w=0;
	while (w<9){	//Mandar parametros del Analizador
		myFile.print(" ");
		if (eastron_read[w]==0){
			myFile.print("0");
		}else{
			myFile.print(eastron_read[w],1);
		}
		w++;
	}
	while (w<12){	//Mandar parametros del Analizador
		myFile.print(" ");
		if (eastron_read[w]==0){
			myFile.print("0");
		}else{
			myFile.print(eastron_read[w],2);
		}
		w++;
	}
	
	w=0;
	while (w<4){
		byte type_temp=screen_type[w+20];
		myFile.print(" ");
		
		if (send_packet_type == 1)
		{
			myFile.print(type_temp);
			myFile.print(":");
		}
		
		if (((type_temp>=35)&&(type_temp<=38))||((type_temp>=135)&&(type_temp<=138))){
			if (Q_flow_opto[w]==0){
				myFile.print("0");
			}else{
				myFile.print(Q_flow_opto[w],2);
			}
		}else{
			if (ain_values_corrected[w]==0){
				myFile.print("0");
			}else{
				myFile.print(ain_values_corrected[w],2);
			}
		}
		w++;
	}
	
	w=0;
	while (w<8)
	{
		byte type_temp=screen_type[26+w];		
		myFile.print(" ");
		
		if (send_packet_type == 1)
		{
			myFile.print(type_temp);
			myFile.print(":");
		}
		
		if (type_temp==1)	//Input
		{
			if (Indio.digitalRead(1+w)==1)
				myFile.print("1");
			else
				myFile.print("0");
		}
		
		else if (type_temp==2)	//Output
		{
			if (1==bitRead(io_digital, w))
				myFile.print("1");
			else
				myFile.print("0");
		}
		else if ((type_temp>=35)&&(type_temp<=38))
		{
			myFile.print(Q_flow_reed[w],2);
		}
		else
		{
			myFile.print("0");
		}
		w++;
	}
	
	w=0;
	while (w<2)
	{
		byte type_temp=screen_type[w+24];	
		
		myFile.print(" ");

		if (send_packet_type == 1)
		{
			myFile.print(type_temp);
			myFile.print(":");
		}
		
		if (type_temp>0)
		{
			if (analog_out[w]==0){
				myFile.print("0");
			}else{
				myFile.print(analog_out[w],2);
			}
		}
		else
			myFile.print("0");
		w++;
	}
	
		
	if (adam_enabled==1)
	{
		
		w=0;
		while (w<8){
			
			byte type_temp=screen_type[w+40];	
			
			myFile.print(" ");
			if (send_packet_type == 1)
			{
				myFile.print(type_temp);
				myFile.print(":");
			}
			
			if (ain_values_corrected[w+4]==0){
				myFile.print("0");
			}else{
				myFile.print(ain_values_corrected[w+4],2);
			}
			w++;
		}
	}
	
	w=0;
	byte send_tuf=0, send_temp=0;
	while (w<4)
	{
		send_tuf=send_tuf + bitRead(modbus_enabled[0], w+4 );
		send_temp=send_temp + bitRead(modbus_enabled[1], w );
		w++;
	}
	
	
	if (send_tuf > 0)
	{
		w=0;
		while (w<4)
		{
			myFile.print(" ");
			if (TUF_flow[w]==0){
				myFile.print("0");
			}else{
				myFile.print(TUF_flow[w],1);
			}
			w++;
		}
	}
	
	if (send_temp > 0)
	{
		w=0;
		while (w<4)
		{
			myFile.print(" ");
			if (temperaturas[w*2]==0){
				myFile.print("0");
			}else{
				myFile.print(temperaturas[w*2],1);
			}
			myFile.print(" ");
			if (temperaturas[w*2+1]==0){
				myFile.print("0");
			}else{
				myFile.print(temperaturas[w*2+1],1);
			}
			w++;
		}
	}
	myFile.println("");
	myFile.close();
	
	display_bottom_status=17;
	draw_screen();
	// close the file:
	
	
}

void loop()
{
  ciclos++;
  Watchdog.reset();	//Resetear el Watchdog
  next_FSM();  	//Definir cual es el siguiente estado del FSM
  connection_status = modbus_update(packets);	//Actualizar los valores que se obtienen mediante Modbus RS-485
  ReadAin();	//Leer las entradas analógicas
  a_out();
  d_out();
  ModbusToFloat();	//Convertir los valores de Modbus en valores float
  draw_screen(); 
  pulse_meters();
  count();
  for (int i=0; i < 8; i++){ //Write pulse counters into FRAM
		u_2.ul=pulse_counter[i];
		FRAMWrite(0x10+4*i, u_2.asbyte, 4);
	}
  
  char buff[512];  int len = 512;	
  webserver.processConnection(buff, &len);	//Atender el servidor HTTP
    
  parsearUDP();
  
  
  
  alarm_relay_out();	//Gestionar la lógica para activar el relé de Alarma
  
  //Eventos de tiempo:
  time_temp=millis();	//Conseguir el timepo actual
    
  
  if ((time_temp-purge_time) > 86400000)
  {
	purge_time=time_temp;
	purga_compresor();
  }

  if (((time_temp-timepost) > post_interval)) //Post UDP and read opto input
  {
	switch (send_packet_type){
	case 0:
		post_UDP();
		break;	
	case 1:
		post_UDP();
		break;	
	
	case 2:
		post_EMON();
		break;	
	}
	
	first_loop=1;
	timepost=time_temp;
	
	//Read Opto each call one optoreader
	byte kw1=0,kw2=0;
	while (kw1==0&&kw2<4)
	{
		
		if ((screen_type[opto_round+20]>=35)&&(screen_type[opto_round+20]<=38))
		{
			float t1,t2,t3;
			EEPROM.get( 0x50+16*opto_round, t1 );
			EEPROM.get( 0x54+16*opto_round, t2 );
			EEPROM.get( 0x58+16*opto_round, t3 );
			if (t1>5)
				t1=5;
			kw1=1;
			display_bottom_status=3;
			draw_screen();
			Q_flow_opto[opto_round]=ReadPulseInput(opto_round+1, t1, t2, t3);
			opto_round++;
			if (opto_round>=4)
				opto_round=0;
			
		}
		else
		{
			opto_round++;
			if (opto_round>=4)
				opto_round=0;
		}
		
		kw2++;
	}
	

  }

  if ((time_temp-timepost_sd) > 2000) //Si han pasado mas de 1.5 segundos desde el post, limpiar el mensaje de pantalla
  {
	  Post_SD();	  
	  timepost_sd=time_temp;
  }
  
  if ((time_temp-timepost) > 1500) //Si han pasado mas de 1.5 segundos desde el post, limpiar el mensaje de pantalla
  {
	  display_bottom_status=0;
  }

}