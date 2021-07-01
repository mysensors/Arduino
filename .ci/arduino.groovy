#!groovy
def buildArduino(config, String buildFlags, String sketch, String key) {
	def root              = '/opt/arduino-1.8.13/'	
	def build_path        = 'build'
	def build_path_cmd    = ' -build-path '+build_path+' '
	if (config.nightly_arduino_ide)
	{
		root = '/opt/arduino-nightly/'
		// patch for nightly arduino-builder
	}
	def jenkins_root      = '/var/lib/jenkins/'
	def builder           = root+'arduino-builder'
	def standard_args     = ' -warnings=all' //-verbose=true
	def builder_specifics = ' -hardware '+root+'hardware -tools '+root+'hardware/tools/avr -tools '+
		root+'tools-builder -built-in-libraries '+root+'libraries'
	def jenkins_packages  = jenkins_root+'.arduino15/packages'
	def site_specifics    = ' -hardware '+jenkins_packages+' -tools '+jenkins_packages
	def repo_specifics    = ' -hardware hardware -libraries . '
	def build_cmd         = builder+standard_args+builder_specifics+site_specifics+repo_specifics+build_path_cmd+buildFlags
	sh """#!/bin/bash
				printf "\\e[1m\\e[32mBuilding \\e[34m${sketch} \\e[0musing \\e[1m\\e[36m${build_cmd}\\e[0m\\n"
				if [ -d ${build_path} ]; then rm -r ${build_path}; fi
				mkdir ${build_path}
				${build_cmd} ${sketch} 2>> compiler_${key}.log"""
}

def parseWarnings(String key) {
	warnings canResolveRelativePaths: false, canRunOnFailed: true, categoriesPattern: '',
 		defaultEncoding: '',
 		excludePattern: '''.*/EEPROM\\.h,.*/Dns\\.cpp,.*/socket\\.cpp,.*/util\\.h,.*/Servo\\.cpp,
 											 .*/Adafruit_NeoPixel\\.cpp,.*/UIPEthernet.*,.*/SoftwareSerial\\.cpp,
 											 .*/pins_arduino\\.h,.*/Stream\\.cpp,.*/USBCore\\.cpp,.*/libraries/Wire/.*,
 											 .*/hardware/STM32F1.*,.*/hardware/esp8266.*,.*/hardware/esp32.*,.*/hardware/samd.*,
											 .*/libraries/SD/.*,.*/libraries/Ethernet/.*''',

 		healthy: '', includePattern: '', messagesPattern: '',
 		parserConfigurations: [[parserName: 'Arduino/AVR', pattern: 'compiler_'+key+'.log']],
 		unHealthy: '', unstableNewAll: '0', unstableTotalAll: '0'
	sh """#!/bin/bash
				echo "Compiler warnings/errors:"
				printf "\\e[101m"
				cat compiler_${key}.log
				printf "\\e[0m"
				rm compiler_${key}.log"""
}

def buildMySensorsMicro(config, sketches, String key) {
	def fqbn = '-fqbn=MySensors:avr:MysensorsMicro:cpu=1Mhz'
	config.pr.setBuildStatus(config, 'PENDING', 'Toll gate (MySensorsMicro - '+key+')', 'Building...', '${BUILD_URL}flowGraphTable/')
	try {
		for (sketch = 0; sketch < sketches.size(); sketch++) {
			if (sketches[sketch].path != config.library_root+'examples/GatewayESP8266/GatewayESP8266.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266MQTTClient/GatewayESP8266MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266OTA/GatewayESP8266OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayGSMMQTTClient/GatewayGSMMQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32/GatewayESP32.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32OTA/GatewayESP32OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32MQTTClient/GatewayESP32MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/SensebenderGatewaySerial/SensebenderGatewaySerial.ino') {
				buildArduino(config, fqbn, sketches[sketch].path, key+'_MySensorsMicro')
			}
		}
	} catch (ex) {
		echo "Build failed with: "+ ex.toString()
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (MySensorsMicro - '+key+')', 'Build error', '${BUILD_URL}')
		throw ex
	} finally {
		parseWarnings(key+'_MySensorsMicro')
	}
	if (currentBuild.currentResult == 'UNSTABLE') {
		config.pr.setBuildStatus(config, 'ERROR', 'Toll gate (MySensorsMicro - '+key+')', 'Warnings found', '${BUILD_URL}warnings2Result/new')
		if (config.is_pull_request) {
			error 'Terminated due to warnings found'
		}
	} else if (currentBuild.currentResult == 'FAILURE') {
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (MySensorsMicro - '+key+')', 'Build error', '${BUILD_URL}')
	} else {
		config.pr.setBuildStatus(config, 'SUCCESS', 'Toll gate (MySensorsMicro - '+key+')', 'Pass', '')
	}
}

def buildMySensorsGw(config, sketches, String key) {
	def fqbn = '-fqbn MySensors:samd:mysensors_gw_native'
	config.pr.setBuildStatus(config, 'PENDING', 'Toll gate (MySensorsGW - '+key+')', 'Building...', '${BUILD_URL}flowGraphTable/')
	try {
		for (sketch = 0; sketch < sketches.size(); sketch++) {
			if (sketches[sketch].path != config.library_root+'examples/BatteryPoweredSensor/BatteryPoweredSensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266/GatewayESP8266.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266MQTTClient/GatewayESP8266MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266OTA/GatewayESP8266OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayGSMMQTTClient/GatewayGSMMQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32/GatewayESP32.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32OTA/GatewayESP32OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32MQTTClient/GatewayESP32MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewaySerialRS485/GatewaySerialRS485.ino' &&
					sketches[sketch].path != config.library_root+'examples/MotionSensorRS485/MotionSensorRS485.ino') {
				buildArduino(config, fqbn, sketches[sketch].path, key+'_MySensorsGW')
			}
		}
	} catch (ex) {
		echo "Build failed with: "+ ex.toString()
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (MySensorsGW - '+key+')', 'Build error', '${BUILD_URL}')
		throw ex
	} finally {
		parseWarnings(key+'_MySensorsGW')
	}
	if (currentBuild.currentResult == 'UNSTABLE') {
		config.pr.setBuildStatus(config, 'ERROR', 'Toll gate (MySensorsGW - '+key+')', 'Warnings found', '${BUILD_URL}warnings2Result/new')
		if (config.is_pull_request) {
			error 'Terminated due to warnings found'
		}
	} else if (currentBuild.currentResult == 'FAILURE') {
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (MySensorsGW - '+key+')', 'Build error', '${BUILD_URL}')
	} else {
		config.pr.setBuildStatus(config, 'SUCCESS', 'Toll gate (MySensorsGW - '+key+')', 'Pass', '')
	}
}

def buildArduinoUno(config, sketches, String key) {
	def fqbn = '-fqbn arduino:avr:uno'
	config.pr.setBuildStatus(config, 'PENDING', 'Toll gate (Arduino Uno - '+key+')', 'Building...', '${BUILD_URL}flowGraphTable/')
	try {
		for (sketch = 0; sketch < sketches.size(); sketch++) {
			if (sketches[sketch].path != config.library_root+'examples/GatewayESP8266/GatewayESP8266.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266MQTTClient/GatewayESP8266MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266OTA/GatewayESP8266OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32/GatewayESP32.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32OTA/GatewayESP32OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32MQTTClient/GatewayESP32MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/SensebenderGatewaySerial/SensebenderGatewaySerial.ino') {
				buildArduino(config, fqbn, sketches[sketch].path, key+'_ArduinoUno')
			}
		}
	} catch (ex) {
		echo "Build failed with: "+ ex.toString()
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (Arduino Uno - '+key+')', 'Build error', '${BUILD_URL}')
		throw ex
	} finally {
		parseWarnings(key+'_ArduinoUno')
	}
	if (currentBuild.currentResult == 'UNSTABLE') {
		config.pr.setBuildStatus(config, 'ERROR', 'Toll gate (Arduino Uno - '+key+')', 'Warnings found', '${BUILD_URL}warnings2Result/new')
		if (config.is_pull_request) {
			error 'Terminated due to warnings found'
		}
	} else if (currentBuild.currentResult == 'FAILURE') {
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (Arduino Uno - '+key+')', 'Build error', '${BUILD_URL}')
	} else {
		config.pr.setBuildStatus(config, 'SUCCESS', 'Toll gate (Arduino Uno - '+key+')', 'Pass', '')
	}
}

def buildArduinoMega(config, sketches, String key) {
	def fqbn = '-fqbn arduino:avr:mega:cpu=atmega2560'
	config.pr.setBuildStatus(config, 'PENDING', 'Toll gate (Arduino Mega - '+key+')', 'Building...', '${BUILD_URL}flowGraphTable/')
	try {
		for (sketch = 0; sketch < sketches.size(); sketch++) {
			if (sketches[sketch].path != config.library_root+'examples/GatewayESP8266/GatewayESP8266.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266MQTTClient/GatewayESP8266MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266OTA/GatewayESP8266OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32/GatewayESP32.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32OTA/GatewayESP32OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32MQTTClient/GatewayESP32MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/SensebenderGatewaySerial/SensebenderGatewaySerial.ino') {
				buildArduino(config, fqbn, sketches[sketch].path, key+'_ArduinoMega')
			}
		}
	} catch (ex) {
		echo "Build failed with: "+ ex.toString()
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (Arduino Mega - '+key+')', 'Build error', '${BUILD_URL}')
		throw ex
	} finally {
		parseWarnings(key+'_ArduinoMega')
	}
	if (currentBuild.currentResult == 'UNSTABLE') {
		config.pr.setBuildStatus(config, config, 'ERROR', 'Toll gate (Arduino Mega - '+key+')', 'Warnings found', '${BUILD_URL}warnings2Result/new')
		if (config.is_pull_request) {
			error 'Terminated due to warnings found'
		}
	} else if (currentBuild.currentResult == 'FAILURE') {
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (Arduino Mega - '+key+')', 'Build error', '${BUILD_URL}')
	} else {
		config.pr.setBuildStatus(config, 'SUCCESS', 'Toll gate (Arduino Mega - '+key+')', 'Pass', '')
	}
}

def buildSTM32F1(config, sketches, String key) {
	def fqbn = '-fqbn stm32duino:STM32F1:genericSTM32F103C:device_variant=STM32F103C8,upload_method=DFUUploadMethod,cpu_speed=speed_72mhz,opt=osstd'
	config.pr.setBuildStatus(config, 'PENDING', 'Toll gate (STM32F1 - '+key+')', 'Building...', '${BUILD_URL}flowGraphTable/')
	try {
		for (sketch = 0; sketch < sketches.size(); sketch++) {
			if (sketches[sketch].path != config.library_root+'examples/GatewayESP8266/GatewayESP8266.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266MQTTClient/GatewayESP8266MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266OTA/GatewayESP8266OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32/GatewayESP32.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32OTA/GatewayESP32OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32MQTTClient/GatewayESP32MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/SensebenderGatewaySerial/SensebenderGatewaySerial.ino') {
				buildArduino(config, fqbn, sketches[sketch].path, key+'_STM32F1')
			}
		}
	} catch (ex) {
		echo "Build failed with: "+ ex.toString()
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (STM32F1 - '+key+')', 'Build error', '${BUILD_URL}')
		throw ex
	} finally {
		parseWarnings(key+'_STM32F1')
	}
	if (currentBuild.currentResult == 'UNSTABLE') {
		config.pr.setBuildStatus(config, 'ERROR', 'Toll gate (STM32F1 - '+key+')', 'Warnings found', '${BUILD_URL}warnings2Result/new')
		if (config.is_pull_request) {
			error 'Terminated due to warnings found'
		}
	} else if (currentBuild.currentResult == 'FAILURE') {
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (STM32F1 - '+key+')', 'Build error', '${BUILD_URL}')
	} else {
		config.pr.setBuildStatus(config, 'SUCCESS', 'Toll gate (STM32F1 - '+key+')', 'Pass', '')
	}
}

def buildESP8266(config, sketches, String key) {
	def fqbn = '-fqbn=esp8266:esp8266:generic:xtal=80,vt=flash,exception=disabled,ResetMethod=ck,CrystalFreq=26,FlashFreq=40,FlashMode=dout,eesz=512K,led=2,ip=lm2f,dbg=Disabled,lvl=None____,wipe=none,baud=115200' 
	config.pr.setBuildStatus(config, 'PENDING', 'Toll gate (ESP8266 - '+key+')', 'Building...', '${BUILD_URL}flowGraphTable/')
	try {
		for (sketch = 0; sketch < sketches.size(); sketch++) {
			if (sketches[sketch].path != config.library_root+'examples/BatteryPoweredSensor/BatteryPoweredSensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/CO2Sensor/CO2Sensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/DustSensorDSM/DustSensorDSM.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewaySerialRS485/GatewaySerialRS485.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayW5100/GatewayW5100.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayW5100MQTTClient/GatewayW5100MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayGSMMQTTClient/GatewayGSMMQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32/GatewayESP32.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32OTA/GatewayESP32OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32MQTTClient/GatewayESP32MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/MotionSensorRS485/MotionSensorRS485.ino' &&
					sketches[sketch].path != config.library_root+'examples/SensebenderGatewaySerial/SensebenderGatewaySerial.ino' &&
					sketches[sketch].path != config.library_root+'examples/SoilMoistSensor/SoilMoistSensor.ino') {
				buildArduino(config, fqbn, sketches[sketch].path, key+'_ESP8266')
			}
		}
	} catch (ex) {
		echo "Build failed with: "+ ex.toString()
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (ESP8266 - '+key+')', 'Build error', '${BUILD_URL}')
		throw ex
	} finally {
		parseWarnings(key+'_ESP8266')
	}
	if (currentBuild.currentResult == 'UNSTABLE') {
		config.pr.setBuildStatus(config, 'ERROR', 'Toll gate (ESP8266 - '+key+')', 'Warnings found', '${BUILD_URL}warnings2Result/new')
		if (config.is_pull_request) {
			error 'Terminated due to warnings found'
		}
	} else if (currentBuild.currentResult == 'FAILURE') {
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (ESP8266 - '+key+')', 'Build error', '${BUILD_URL}')
	} else {
		config.pr.setBuildStatus(config, 'SUCCESS', 'Toll gate (ESP8266 - '+key+')', 'Pass', '')
	}
}

def buildESP32(config, sketches, String key) {
	def fqbn = '-fqbn esp32:esp32:esp32:PartitionScheme=default,FlashMode=qio,FlashFreq=80,FlashSize=4M,UploadSpeed=921600,DebugLevel=none -warnings=default'
	config.pr.setBuildStatus(config, 'PENDING', 'Toll gate (ESP32 - '+key+')', 'Building...', '${BUILD_URL}flowGraphTable/')
	try {
		for (sketch = 0; sketch < sketches.size(); sketch++) {
			if (sketches[sketch].path != config.library_root+'examples/BatteryPoweredSensor/BatteryPoweredSensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/BinarySwitchSleepSensor/BinarySwitchSleepSensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/CO2Sensor/CO2Sensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/DustSensor/DustSensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/DustSensorDSM/DustSensorDSM.ino' &&
					sketches[sketch].path != config.library_root+'examples/EnergyMeterPulseSensor/EnergyMeterPulseSensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/LightSensor/LightSensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/LogOTANode/LogOTANode.ino' &&
					sketches[sketch].path != config.library_root+'examples/MotionSensor/MotionSensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/MotionSensorRS485/MotionSensorRS485.ino' &&
					sketches[sketch].path != config.library_root+'examples/PassiveNode/PassiveNode.ino' &&		
					sketches[sketch].path != config.library_root+'examples/GatewaySerialRS485/GatewaySerialRS485.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayW5100/GatewayW5100.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayW5100MQTTClient/GatewayW5100MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayGSMMQTTClient/GatewayGSMMQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266/GatewayESP8266.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266MQTTClient/GatewayESP8266MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266OTA/GatewayESP8266OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/SensebenderGatewaySerial/SensebenderGatewaySerial.ino' &&
					sketches[sketch].path != config.library_root+'examples/MotionSensorRS485/MotionSensorRS485.ino' &&
					sketches[sketch].path != config.library_root+'examples/SoilMoistSensor/SoilMoistSensor.ino') {
				buildArduino(config, fqbn, sketches[sketch].path, key+'_ESP32')
			}
		}
	} catch (ex) {
		echo "Build failed with: "+ ex.toString()
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (ESP32 - '+key+')', 'Build error', '${BUILD_URL}')
		throw ex
	} finally {
		parseWarnings(key+'_ESP32')
	}
	if (currentBuild.currentResult == 'UNSTABLE') {
		config.pr.setBuildStatus(config, 'ERROR', 'Toll gate (ESP32 - '+key+')', 'Warnings found', '${BUILD_URL}warnings2Result/new')
		if (config.is_pull_request) {
			error 'Terminated due to warnings found'
		}
	} else if (currentBuild.currentResult == 'FAILURE') {
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (ESP32 - '+key+')', 'Build error', '${BUILD_URL}')
	} else {
		config.pr.setBuildStatus(config, 'SUCCESS', 'Toll gate (ESP32 - '+key+')', 'Pass', '')
	}
} 

def buildnRF5(config, sketches, String key) {
	def fqbn = '-fqbn sandeepmistry:nRF5:Generic_nRF52832:softdevice=none,lfclk=lfxo'
	config.pr.setBuildStatus(config, 'PENDING', 'Toll gate (nRF5 - '+key+')', 'Building...', '${BUILD_URL}flowGraphTable/')
	try {
		for (sketch = 0; sketch < sketches.size(); sketch++) {
			if (sketches[sketch].path != config.library_root+'examples/BatteryPoweredSensor/BatteryPoweredSensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/CO2Sensor/CO2Sensor.ino' &&
					sketches[sketch].path != config.library_root+'examples/DustSensorDSM/DustSensorDSM.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266/GatewayESP8266.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266MQTTClient/GatewayESP8266MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayGSMMQTTClient/GatewayGSMMQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP8266OTA/GatewayESP8266OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32/GatewayESP32.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32OTA/GatewayESP32OTA.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayESP32MQTTClient/GatewayESP32MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewaySerialRS485/GatewaySerialRS485.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayW5100/GatewayW5100.ino' &&
					sketches[sketch].path != config.library_root+'examples/GatewayW5100MQTTClient/GatewayW5100MQTTClient.ino' &&
					sketches[sketch].path != config.library_root+'examples/MotionSensorRS485/MotionSensorRS485.ino' &&
					sketches[sketch].path != config.library_root+'examples/SensebenderGatewaySerial/SensebenderGatewaySerial.ino') {
				buildArduino(config, fqbn, sketches[sketch].path, key+'_nRF5')
			}
		}
	} catch (ex) {
		echo "Build failed with: "+ ex.toString()
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (nRF5 - '+key+')', 'Build error', '${BUILD_URL}')
		throw ex
	} finally {
		parseWarnings(key+'_nRF5')
	}
	if (currentBuild.currentResult == 'UNSTABLE') {
		config.pr.setBuildStatus(config, 'ERROR', 'Toll gate (nRF5 - '+key+')', 'Warnings found', '${BUILD_URL}warnings2Result/new')
		if (config.is_pull_request) {
			error 'Terminated due to warnings found'
		}
	} else if (currentBuild.currentResult == 'FAILURE') {
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (nRF5 - '+key+')', 'Build error', '${BUILD_URL}')
	} else {
		config.pr.setBuildStatus(config, 'SUCCESS', 'Toll gate (nRF5 - '+key+')', 'Pass', '')
	}
}

def buildnRF52832(config, sketches, String key) {
	def fqbn = '-fqbn=MySensors:nRF5:MyBoard_nRF52832:bootcode=none,lfclk=lfxo,reset=notenable'
	config.pr.setBuildStatus(config, 'PENDING', 'Toll gate (nRF52832 - '+key+')', 'Building...', '${BUILD_URL}flowGraphTable/')
	try {
		buildArduino(config, fqbn, 'hardware/MySensors/nRF5/libraries/MyBoardNRF5/examples/MyBoardNRF5/MyBoardNRF5.ino', key+'_nRF52832')
	} catch (ex) {
		echo "Build failed with: "+ ex.toString()
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (nRF52832 - '+key+')', 'Build error', '${BUILD_URL}')
		throw ex
	} finally {
		parseWarnings(key+'_nRF52832')
	}
	if (currentBuild.currentResult == 'UNSTABLE') {
		config.pr.setBuildStatus(config, 'ERROR', 'Toll gate (nRF52832 - '+key+')', 'Warnings found', '${BUILD_URL}warnings2Result/new')
		if (config.is_pull_request) {
			error 'Terminated due to warnings found'
		}
	} else if (currentBuild.currentResult == 'FAILURE') {
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (nRF52832 - '+key+')', 'Build error', '${BUILD_URL}')
	} else {
		config.pr.setBuildStatus(config, 'SUCCESS', 'Toll gate (nRF52832 - '+key+')', 'Pass', '')
	}
}

def buildnRF51822(config, sketches, String key) {
	def fqbn = '-fqbn=MySensors:nRF5:MyBoard_nRF51822:chip=xxaa,bootcode=none,lfclk=lfxo'
	config.pr.setBuildStatus(config, 'PENDING', 'Toll gate (nRF51822 - '+key+')', 'Building...', '${BUILD_URL}flowGraphTable/')
	try {
		buildArduino(config, fqbn, 'hardware/MySensors/nRF5/libraries/MyBoardNRF5/examples/MyBoardNRF5/MyBoardNRF5.ino', key+'_nRF51822')
	} catch (ex) {
		echo "Build failed with: "+ ex.toString()
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (nRF51822 - '+key+')', 'Build error', '${BUILD_URL}')
		throw ex
	} finally {
		parseWarnings(key+'_nRF51822')
	}
	if (currentBuild.currentResult == 'UNSTABLE') {
		config.pr.setBuildStatus(config, 'ERROR', 'Toll gate (nRF51822 - '+key+')', 'Warnings found', '${BUILD_URL}warnings2Result/new')
		if (config.is_pull_request) {
			error 'Terminated due to warnings found'
		}
	} else if (currentBuild.currentResult == 'FAILURE') {
		config.pr.setBuildStatus(config, 'FAILURE', 'Toll gate (nRF51822 - '+key+')', 'Build error', '${BUILD_URL}')
	} else {
		config.pr.setBuildStatus(config, 'SUCCESS', 'Toll gate (nRF51822 - '+key+')', 'Pass', '')
	}
}

return this