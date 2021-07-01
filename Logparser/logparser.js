function copyTextToClipboard(text) {
	var textArea = document.createElement("textarea");

	//
	// *** This styling is an extra step which is likely not required. ***
	//
	// Why is it here? To ensure:
	// 1. the element is able to have focus and selection.
	// 2. if element was to flash render it has minimal visual impact.
	// 3. less flakyness with selection and copying which **might** occur if
	//    the textarea element is not visible.
	//
	// The likelihood is the element won't even render, not even a flash,
	// so some of these are just precautions. However in IE the element
	// is visible whilst the popup box asking the user for permission for
	// the web page to copy to the clipboard.
	//

	// Place in top-left corner of screen regardless of scroll position.
	textArea.style.position = 'fixed';
	textArea.style.top = 0;
	textArea.style.left = 0;

	// Ensure it has a small width and height. Setting to 1px / 1em
	// doesn't work as this gives a negative w/h on some browsers.
	textArea.style.width = '2em';
	textArea.style.height = '2em';

	// We don't need padding, reducing the size if it does flash render.
	textArea.style.padding = 0;

	// Clean up any borders.
	textArea.style.border = 'none';
	textArea.style.outline = 'none';
	textArea.style.boxShadow = 'none';

	// Avoid flash of white box if rendered for any reason.
	textArea.style.background = 'transparent';


	textArea.value = text;

	document.body.appendChild(textArea);

	textArea.select();

	try {
		var successful = document.execCommand('copy');
		var msg = successful ? 'successful' : 'unsuccessful';
		console.log('Copying text command was ' + msg);
	} catch (err) {
		console.log('Oops, unable to copy');
	}

	document.body.removeChild(textArea);
}

var types = {
"presentation":[
"S_DOOR",
"S_MOTION",
"S_SMOKE",
"S_BINARY",
"S_DIMMER",
"S_COVER",
"S_TEMP",
"S_HUM",
"S_BARO",
"S_WIND",
"S_RAIN",
"S_UV",
"S_WEIGHT",
"S_POWER",
"S_HEATER",
"S_DISTANCE",
"S_LIGHT_LEVEL",
"S_ARDUINO_NODE",
"S_ARDUINO_REPEATER_NODE",
"S_LOCK",
"S_IR",
"S_WATER",
"S_AIR_QUALITY",
"S_CUSTOM",
"S_DUST",
"S_SCENE_CONTROLLER",
"S_RGB_LIGHT",
"S_RGBW_LIGHT",
"S_COLOR_SENSOR",
"S_HVAC",
"S_MULTIMETER",
"S_SPRINKLER",
"S_WATER_LEAK",
"S_SOUND",
"S_VIBRATION",
"S_MOISTURE",
"S_INFO",
"S_GAS",
"S_GPS",
"S_WATER_QUALITY"
],
"internal": [
"I_BATTERY_LEVEL",
"I_TIME",
"I_VERSION",
"I_ID_REQUEST",
"I_ID_RESPONSE",
"I_INCLUSION_MODE",
"I_CONFIG",
"I_FIND_PARENT_REQUEST",
"I_FIND_PARENT_RESPONSE",
"I_LOG_MESSAGE",
"I_CHILDREN",
"I_SKETCH_NAME",
"I_SKETCH_VERSION",
"I_REBOOT",
"I_GATEWAY_READY",
"I_SIGNING_PRESENTATION",
"I_NONCE_REQUEST",
"I_NONCE_RESPONSE",
"I_HEARTBEAT_REQUEST",
"I_PRESENTATION",
"I_DISCOVER_REQUEST",
"I_DISCOVER_RESPONSE",
"I_HEARTBEAT_RESPONSE",
"I_LOCKED",
"I_PING",
"I_PONG",
"I_REGISTRATION_REQUEST",
"I_REGISTRATION_RESPONSE",
"I_DEBUG",
"I_SIGNAL_REPORT_REQUEST",
"I_SIGNAL_REPORT_REVERSE",
"I_SIGNAL_REPORT_RESPONSE",
"I_PRE_SLEEP_NOTIFICATION",
"I_POST_SLEEP_NOTIFICATION"
],
"subtype":[
"V_TEMP",
"V_HUM",
"V_STATUS",
"V_PERCENTAGE",
"V_PRESSURE",
"V_FORECAST",
"V_RAIN",
"V_RAINRATE",
"V_WIND",
"V_GUST",
"V_DIRECTION",
"V_UV",
"V_WEIGHT",
"V_DISTANCE",
"V_IMPEDANCE",
"V_ARMED",
"V_TRIPPED",
"V_WATT",
"V_KWH",
"V_SCENE_ON",
"V_SCENE_OFF",
"V_HVAC_FLOW_STATE",
"V_HVAC_SPEED",
"V_LIGHT_LEVEL",
"V_VAR1",
"V_VAR2",
"V_VAR3",
"V_VAR4",
"V_VAR5",
"V_UP",
"V_DOWN",
"V_STOP",
"V_IR_SEND",
"V_IR_RECEIVE",
"V_FLOW",
"V_VOLUME",
"V_LOCK_STATUS",
"V_LEVEL",
"V_VOLTAGE",
"V_CURRENT",
"V_RGB",
"V_RGBW",
"V_ID",
"V_UNIT_PREFIX",
"V_HVAC_SETPOINT_COOL",
"V_HVAC_SETPOINT_HEAT",
"V_HVAC_FLOW_MODE",
"V_TEXT",
"V_CUSTOM",
"V_POSITION",
"V_IR_RECORD",
"V_PH",
"V_ORP",
"V_EC",
"V_VAR",
"V_VA",
"V_POWER_FACTOR"
],
"stream":[
"ST_FIRMWARE_CONFIG_REQUEST",
"ST_FIRMWARE_CONFIG_RESPONSE",
"ST_FIRMWARE_REQUEST",
"ST_FIRMWARE_RESPONSE",
"ST_SOUND",
"ST_IMAGE"],
	command: [
"PRESENTATION",
"SET",
"REQ",
"INTERNAL",
"STREAM"
	],
"payloadtype":[
"P_STRING",
"P_BYTE",
"P_INT16",
"P_UINT16",
"P_LONG32",
"P_ULONG32",
"P_CUSTOM",
"P_FLOAT32"
]};

//mysgw: Client 0: 0;0;3;0;18;PING
var rprefix =  "(?:\\d+ )?(?:mysgw: )?(?:Client 0: )?";
var prefilter = /(?=[!?]?(MCO|TSM|TSF|THA|SGN))(?<result>.*)/;
var match = [

	// MySensors core

	{ re: "MCO:BGN:INIT CP=([^,]+)", d: "Core initialization with capabilities <b>$1</b>" },
	{ re: "MCO:BGN:INIT (\\w+),CP=([^,]+),VER=(.*)", d: "Core initialization of <b>$1</b>, with capabilities <b>$2</b>, library version <b>$3</b>" },
	{ re: "MCO:BGN:INIT (\\w+),CP=([^,]+),REL=(.*),VER=(.*)", d: "Core initialization of <b>$1</b>, with capabilities <b>$2</b>, library version <b>$4</b>, release <b>$3</b>" },
	{ re: "MCO:BGN:INIT (\\w+),CP=([^,]+),FQ=(\\d+),REL=(.*),VER=(.*)", d: "Core initialization of <b>$1</b>, with capabilities <b>$2</b>, CPU frequency <b>$3</b> MHz, library version <b>$5</b>, release <b>$4</b>" },
	{ re: "MCO:BGN:BFR", d: "Callback before()" },
	{ re: "MCO:BGN:STP", d: "Callback setup()" },
	{ re: "MCO:BGN:INIT OK,TSP=(.*)", d: "Core initialized, transport status <b>$1</b>, (1=initialized, 0=not initialized, NA=not available)" },
	{ re: "MCO:BGN:NODE UNLOCKED", d: "Node successfully unlocked (see signing chapter)" },
	{ re: "!MCO:BGN:TSP FAIL", d: "Transport initialization failed" },
	{ re: "MCO:REG:REQ", d: "Registration request" },
	{ re: "MCO:REG:NOT NEEDED", d: "No registration needed (i.e. GW)" },
	{ re: "!MCO:SND:NODE NOT REG", d: "Node is not registered, cannot send message" },
	{ re: "MCO:PIM:NODE REG=(\\d+)", d: "Registration response received, registration status <b>$1</b>" },
	{ re: "MCO:PIM:ROUTE N=(\\d+),R=(\\d+)", d: "Routing table, messages to node <b>$1</b> are routed via node <b>$2</b>" },
	{ re: "MCO:SLP:MS=(\\d+),SMS=(\\d+),I1=(\\d+),M1=(\\d+),I2=(\\d+),M2=(\\d+)", d: "Sleep node, duration <b>$1</b> ms, SmartSleep=<b>$2</b>, Int1=<b>$3</b>, Mode1=<b>$4</b>, Int2=<b>$5</b>, Mode2=<b>$6</b>" },
	{ re: "MCO:SLP:MS=(\\d+)", d: "Sleep node, duration <b>$1</b> ms" },
	{ re: "MCO:SLP:TPD", d: "Sleep node, powerdown transport" },
	{ re: "MCO:SLP:WUP=(-?\\d+)", d: "Node woke-up, reason/IRQ=<b>$1</b> (-2=not possible, -1=timer, >=0 IRQ)" },
	{ re: "!MCO:SLP:FWUPD", d: "Sleeping not possible, FW update ongoing" },
	{ re: "!MCO:SLP:REP", d: "Sleeping not possible, repeater feature enabled" },
	{ re: "!MCO:SLP:TNR", d: " Transport not ready, attempt to reconnect until timeout" },
	{ re: "MCO:NLK:NODE LOCKED. UNLOCK: GND PIN (\\d+) AND RESET", d: "Node locked during booting, see signing documentation for additional information" },
	{ re: "MCO:NLK:TPD", d: "Powerdown transport" },
	{ re: "!MCO:PRO:RC=(-?\\d+)", d: "Recursive call detected in _process(), call level=<b>$1</b>" },
	{ re: "!MCO:WAI:RC=(-?\\d+)", d: "Recursive call detected in wait(), call level=<b>$1</b>" },
	
	// transport state machine
	
	{ re: "TSM:INIT", d: "Transition to <b>Init</b> state" },
	{ re: "TSM:INIT:STATID=(\\d+)", d: "Init static node id <b>$1</b>" },
	{ re: "TSM:INIT:TSP OK", d: "Transport device configured and fully operational" },
	{ re: "TSM:INIT:GW MODE", d: "Node is set up as GW, thus omitting ID and findParent states" },
	{ re: "!TSM:INIT:TSP FAIL", d: "Transport device initialization failed" },
	{ re: "TSM:FPAR", d: "Transition to <b>Find Parent</b> state" },
	{ re: "TSM:FPAR:STATP=(\\d+)", d: "Static parent <b>$1</b> has been set, skip finding parent" },
	{ re: "TSM:FPAR:OK", d: "Parent node identified" },
	{ re: "!TSM:FPAR:NO REPLY", d: "No potential parents replied to find parent request" },
	{ re: "!TSM:FPAR:FAIL", d: "Finding parent failed" },
	{ re: "TSM:ID", d: "Transition to <b>Request Id</b> state" },
	{ re: "TSM:ID:OK,ID=(\\d+)", d: "Node id <b>$1</b> is valid" },
	{ re: "TSM:ID:REQ", d: "Request node id from controller" },
	{ re: "!TSM:ID:FAIL", d: "Did not receive a node id from controller. Is your controller connected and correctly configured?" },
	{ re: "!TSM:ID:FAIL,ID=(\\d+)", d: "Id verification failed, <b>$1</b> is invalid" },
	{ re: "TSM:UPL", d: "Transition to <b>Check Uplink</b> state" },
	{ re: "TSM:UPL:OK", d: "Uplink OK, GW returned ping" },
	{ re: "!TSM:UPL:FAIL", d: "Uplink check failed, i.e. GW could not be pinged" },
	{ re: "TSM:READY:NWD REQ", d: "Send transport network discovery request" },
	{ re: "TSM:READY:SRT", d: "Save routing table" },
	{ re: "TSM:READY:ID=(\\d+),PAR=(\\d+),DIS=(\\d+)", d: "Transport ready, node id <b>$1</b>, parent node id <b>$2</b>, distance to GW is <b>$3</b>" },
	{ re: "!TSM:READY:UPL FAIL,SNP", d: "Too many failed uplink transmissions, search new parent" },
	{ re: "!TSM:READY:FAIL,STATP", d: "Too many failed uplink transmissions, static parent enforced" },
	{ re: "TSM:READY", d: "Transition to <b>Ready</b> state" },
	{ re: "TSM:FAIL:DIS", d: "Disable transport" },
	{ re: "TSM:FAIL:CNT=(\\d+)", d: "Transition to <b>Failure</b> state, consecutive failure counter is <b>$1</b>" },
	{ re: "TSM:FAIL:PDT", d: "Power-down transport" },
	{ re: "TSM:FAIL:RE-INIT", d: "Attempt to re-initialize transport" },
	
	// transport support functions
	
	{ re: "TSF:CKU:OK,FCTRL", d: "Uplink OK, flood control prevents pinging GW in too short intervals" },
	{ re: "TSF:CKU:OK", d: "Uplink OK" },
	{ re: "TSF:CKU:DGWC,O=(\\d+),N=(\\d+)", d: "Uplink check revealed changed network topology, old distance <b>$1</b>, new distance <b>$2</b>" },
	{ re: "TSF:CKU:FAIL", d: "No reply received when checking uplink" },
	{ re: "TSF:SID:OK,ID=(\\d+)", d: "Node id <b>$1</b> assigned" },
	{ re: "!TSF:SID:FAIL,ID=(\\d+)", d: "Assigned id <b>$1</b> is invalid" },
	{ re: "TSF:PNG:SEND,TO=(\\d+)", d: "Send ping to destination <b>$1</b>" },
	{ re: "!TSF:PNG:ACTIVE", d: "Ping active, cannot start new ping" },
	{ re: "TSF:WUR:MS=(\\d+)", d: "Wait until transport ready, timeout <b>$1</b>" },
	{ re: "TSF:MSG:ECHO REQ", d: "ECHO message requested" },
	{ re: "TSF:MSG:ECHO", d: "ECHO message, do not proceed but forward to callback" },
	{ re: "TSF:MSG:FPAR RES,ID=(\\d+),D=(\\d+)", d: "Response to find parent request received from node <b>$1</b> with distance <b>$2</b> to GW" },
	{ re: "TSF:MSG:FPAR PREF FOUND", d: "Preferred parent found, i.e. parent defined via MY_PARENT_NODE_ID" },
	{ re: "TSF:MSG:FPAR OK,ID=(\\d+),D=(\\d+)", d: "Find parent response from node <b>$1</b> is valid, distance <b>$2</b> to GW" },
	{ re: "!TSF:MSG:FPAR INACTIVE", d: "Find parent response received, but no find parent request active, skip response" },
	{ re: "TSF:MSG:FPAR REQ,ID=(\\d+)", d: "Find parent request from node <b>$1</b>" },
	{ re: "TSF:MSG:PINGED,ID=(\\d+),HP=(\\d+)", d: "Node pinged by node <b>$1</b> with <b>$2</b> hops" },
	{ re: "TSF:MSG:PONG RECV,HP=(\\d+)", d: "Pinged node replied with <b>$1</b> hops" },
	{ re: "!TSF:MSG:PONG RECV,INACTIVE", d: "Pong received, but !pingActive" },
	{ re: "TSF:MSG:BC", d: "Broadcast message received" },
	{ re: "TSF:MSG:GWL OK", d: "Link to GW ok" },
	{ re: "TSF:MSG:FWD BC MSG", d: "Controlled broadcast message forwarding" },
	{ re: "TSF:MSG:REL MSG", d: "Relay message" },
	{ re: "TSF:MSG:REL PxNG,HP=(\\d+)", d: "Relay PING/PONG message, increment hop counter to <b>$1</b>" },
	{ re: "!TSF:MSG:LEN,(\\d+)!=(\\d+)", d: "Invalid message length, <b>$1</b> (actual) != <b>$2</b> (expected)" },
	{ re: "!TSF:MSG:PVER,(\\d+)!=(\\d+)", d: "Message protocol version mismatch, <b>$1</b> (actual) != <b>$2</b> (expected)" },
	{ re: "!TSF:MSG:SIGN VERIFY FAIL", d: "Signing verification failed" },
	{ re: "!TSF:MSG:REL MSG,NORP", d: "Node received a message for relaying, but node is not a repeater, message skipped" },
	{ re: "!TSF:MSG:SIGN FAIL", d: "Signing message failed" },
	{ re: "!TSF:MSG:GWL FAIL", d: "GW uplink failed" },
	{ re: "!TSF:MSG:ID TK INVALID", d: "Token for ID request invalid" },
	{ re: "TSF:SAN:OK", d: "Sanity check passed" },
	{ re: "!TSF:SAN:FAIL", d: "Sanity check failed, attempt to re-initialize radio" },
	{ re: "TSF:CRT:OK", d: "Clearing routing table successful" },
	{ re: "TSF:LRT:OK", d: "Loading routing table successful" },
	{ re: "TSF:SRT:OK", d: "Saving routing table successful" },
	{ re: "!TSF:RTE:FPAR ACTIVE", d: "Finding parent active, message not sent" },
	{ re: "!TSF:RTE:(\\d+) UNKNOWN", d: "Routing for destination <b>$1</b> unknown, sending message to parent" },
	{ re: "!TSF:RTE:N2N FAIL", d: "Direct node-to-node communication failed - handing over to parent" },
	{ re: "TSF:RTE:N2N DIS", d: "Direct node-to-node communication disabled by configuration" },
	{ re: "TSF:RRT:ROUTE N=(\\d+),R=(\\d+)", d: "Routing table, messages to node (<b>$1</b>) are routed via node (<b>$2</b>)"},
	{ re: "!TSF:SND:TNR", d: "Transport not ready, message cannot be sent" },
	{ re: "TSF:TDI:TSL", d: "Set transport to sleep" },
	{ re: "TSF:TDI:TPD", d: "Power down transport" },
	{ re: "TSF:TRI:TRI", d: "Reinitialise transport" },
	{ re: "TSF:TRI:TSB", d: "Set transport to standby" },
	{ re: "TSF:TRI:TPU", d: "Power up transport" },
	{ re: "TSF:SIR:CMD=(\\d+),VAL=(\\d+)", d: "Get signal report <b>$1</b>, value: <b>$2</b>" },
	{ re: "TSF:MSG:READ,(\\d+)-(\\d+)-(\\d+),s=(\\d+),c=(\\d+),t=(\\d+),pt=(\\d+),l=(\\d+),sg=(\\d+):(.*)", d: "<u><b>Received Message</b></u><br><b>Sender</b>: $1<br><b>Last Node</b>: $2<br><b>Destination</b>: $3<br><b>Sensor Id</b>: $4<br><b>Command</b>: {command:$5}<br><b>Message Type</b>: {type:$5:$6}<br><b>Payload Type</b>: {pt:$7}<br><b>Payload Length</b>: $8<br><b>Signing</b>: $9<br><b>Payload</b>: $10" },
	{ re: "TSF:MSG:SEND,(\\d+)-(\\d+)-(\\d+)-(\\d+),s=(\\d+),c=(\\d+),t=(\\d+),pt=(\\d+),l=(\\d+),sg=(\\d+),ft=(\\d+),st=(\\w+):(.*)", d: "<u><b>Sent Message</b></u><br><b>Sender</b>: $1<br><b>Last Node</b>: $2<br><b>Next Node</b>: $3<br><b>Destination</b>: $4<br><b>Sensor Id</b>: $5<br><b>Command</b>: {command:$6}<br><b>Message Type</b>:{type:$6:$7}<br><b>Payload Type</b>: {pt:$8}<br><b>Payload Length</b>: $9<br><b>Signing</b>: $10<br><b>Failed uplink counter</b>: $11<br><b>Status</b>: $12 (OK=success, NACK=no radio ACK received)<br><b>Payload</b>: $13" },
	{ re: "!TSF:MSG:SEND,(\\d+)-(\\d+)-(\\d+)-(\\d+),s=(\\d+),c=(\\d+),t=(\\d+),pt=(\\d+),l=(\\d+),sg=(\\d+),ft=(\\d+),st=(\\w+):(.*)", d: "<u><b style='color:red'>Sent Message</b></u><br><b>Sender</b>: $1<br><b>Last Node</b>: $2<br><b>Next Node</b>: $3<br><b>Destination</b>: $4<br><b>Sensor Id</b>: $5<br><b>Command</b>: {command:$6}<br><b>Message Type</b>:{type:$6:$7}<br><b>Payload Type</b>: {pt:$8}<br><b>Payload Length</b>: $9<br><b>Signing</b>: $10<br><b>Failed uplink counter</b>: $11<br><b>Status</b>: $12 (OK=success, NACK=no radio ACK received)<br><b>Payload</b>: $13" },
	{ re: "\\?TSF:MSG:SEND,(\\d+)-(\\d+)-(\\d+)-(\\d+),s=(\\d+),c=(\\d+),t=(\\d+),pt=(\\d+),l=(\\d+),sg=(\\d+),ft=(\\d+),st=(\\w+):(.*)", d: "<u><b style='color:orange'>Sent Message without radio ACK</b></u><br><b>Sender</b>: $1<br><b>Last Node</b>: $2<br><b>Next Node</b>: $3<br><b>Destination</b>: $4<br><b>Sensor Id</b>: $5<br><b>Command</b>: {command:$6}<br><b>Message Type</b>:{type:$6:$7}<br><b>Payload Type</b>: {pt:$8}<br><b>Payload Length</b>: $9<br><b>Signing</b>: $10<br><b>Failed uplink counter</b>: $11<br><b>Status</b>: $12 (OK=success, NACK=no radio ACK received)<br><b>Payload</b>: $13" },

	// transport HAL
	
	{ re: "THA:INIT", d: "Initialise transport HAL" },
	{ re: "THA:INIT:PSK=(.*)", d: "Initialise transport HAL, PSK=<b>$1</b>" },
	{ re: "THA:SAD:ADDR=(\\d+)", d: "Set transport address: <b>$1</b>" },
	{ re: "THA:GAD:ADDR=(\\d+)", d: "Get transport address: <b>$1</b>" },
	{ re: "THA:DATA:AVAIL", d: "Transport HAL received data" },
	{ re: "THA:SAN:RES=(\\d+)", d: "Transport sanity check, result=<b>$1</b> (0=NOK, 1=OK)" },
	{ re: "THA:RCV:MSG=(.*)", d: "Message received: <b>$1</b>" },
	{ re: "THA:RCV:DECRYPT", d: "Decrypt message" },
	{ re: "THA:RCV:PLAIN=(.*)", d: "Message plaint text: <b>$1</b>" },
	{ re: "!THA:RCV:PVER=(\\d+)", d: "Message protocol version mismatch: <b>$1</b>" },
	{ re: "!THA:RCV:LEN=(\\d+),EXP=(\\d+)", d: "Invalid message length, actual <b>$1</b>, expected <b>$2</b>" },
	{ re: "THA:RCV:MSG LEN=(\\d+)", d: "Length of received message: <b>$1</b>" },
	{ re: "THA:SND:MSG=(.*)", d: "Send message: <b>$1</b>" },
	{ re: "THA:SND:ENCRYPT", d: "Encrypt message" },
	{ re: "THA:SND:CIP=(.*)", d: "Ciphertext of encrypted message: <b>$1</b>" },
	{ re: "THA:SND:MSG LEN=(\\d+),RES=(\\d+)", d: "Sending message with length=<b>$1</b>, result=<b>$2</b> (0=NOK, 1=OK)" },
	
	// Signing backend

	{ re: "SGN:INI:BND OK", d: "Backend has initialized ok" },
	{ re: "!SGN:INI:BND FAIL", d: "Backend has not initialized ok" },
	{ re: "SGN:PER:OK", d: "Personalization data is ok" },
	{ re: "!SGN:PER:TAMPERED", d: "Personalization data has been tampered" },
	{ re: "SGN:PRE:SGN REQ", d: "Signing required" },
	{ re: "SGN:PRE:SGN REQ,TO=(\\d+)", d: "Tell node <b>$1</b> that we require signing" },
	{ re: "SGN:PRE:SGN REQ,FROM=(\\d+)", d: " Node <b>$1</b> require signing" },
	{ re: "SGN:PRE:SGN NREQ", d: "Signing not required" },
	{ re: "SGN:PRE:SGN REQ,TO=(\\d+)", d: "Tell node <b>$1</b> that we do not require signing" },
	{ re: "SGN:PRE:SGN NREQ,FROM=(\\d+)", d: "Node <b>$1</b> does not require signing" },
	{ re: "!SGN:PRE:SGN NREQ,FROM=(\\d+) REJ", d: "Node <b>$1</b> does not require signing but used to (requirement remain unchanged)" },
	{ re: "SGN:PRE:WHI REQ", d: "Whitelisting required" },
	{ re: "SGN:PRE:WHI REQ;TO=(\\d+)", d: "Tell <b>$1</b> that we require whitelisting" },
	{ re: "SGN:PRE:WHI REQ,FROM=(\\d+)", d: "Node <b>$1</b> require whitelisting" },
	{ re: "SGN:PRE:WHI NREQ", d: " Whitelisting not required" },
	{ re: "SGN:PRE:WHI NREQ,TO=(\\d+)", d: "Tell node <b>$1</b> that we do not require whitelisting" },
	{ re: "SGN:PRE:WHI NREQ,FROM=(\\d+)", d: "Node <b>$1</b> does not require whitelisting" },
	{ re: "!SGN:PRE:WHI NREQ,FROM=(\\d+) REJ", d: "Node <b>$1</b> does not require whitelisting but used to (requirement remain unchanged)" },
	{ re: "SGN:PRE:XMT,TO=(\\d+)", d: "Presentation data transmitted to node <b>$1</b>" },
	{ re: "!SGN:PRE:XMT,TO=(\\d+) FAIL", d: "Presentation data not properly transmitted to node <b>$1</b>" },
	{ re: "SGN:PRE:WAIT GW", d: "Waiting for gateway presentation data" },
	{ re: "!SGN:PRE:VER=(\\d+)", d: "Presentation version <b>$1</b> is not supported" },
	{ re: "SGN:PRE:NSUP", d: "Received signing presentation but signing is not supported" },
	{ re: "SGN:PRE:NSUP,TO=(\\d+)", d: "Informing node <b>$1</b> that we do not support signing" },
	{ re: "SGN:SGN:NCE REQ,TO=(\\d+)", d: "Nonce request transmitted to node <b>$1</b>" },
	{ re: "!SGN:SGN:NCE REQ,TO=(\\d+) FAIL", d: "Nonce request not properly transmitted to node <b>$1</b>" },
	{ re: "!SGN:SGN:NCE TMO", d: "Timeout waiting for nonce" },
	{ re: "SGN:SGN:SGN", d: "Message signed" },
	{ re: "!SGN:SGN:SGN FAIL", d: "Message failed to be signed" },
	{ re: "SGN:SGN:NREQ=(\\d+)", d: "Node <b>$1</b> does not require signed messages" },
	{ re: "SGN:SGN:(\\d+)!=(\\d+) NUS", d: "Will not sign because <b>$1</b> is not <b>$2</b> (repeater)" },
	{ re: "!SGN:SGN:STATE", d: "Security system in a invalid state (personalization data tampered)" },
	{ re: "!SGN:VER:NSG", d: "Message was not signed, but it should have been" },
	{ re: "!SGN:VER:FAIL", d: "Verification failed" },
	{ re: "SGN:VER:OK", d: "Verification succeeded" },
	{ re: "SGN:VER:LEFT=(\\d+)", d: "<b>$1</b> number of failed verifications left in a row before node is locked" },
	{ re: "!SGN:VER:STATE", d: "Security system in a invalid state (personalization data tampered)" },
	{ re: "SGN:SKP:MSG CMD=(\\d+),TYPE=(\\d+)", d: "Message with command <b>$1</b> and type <b>$2</b> does not need to be signed" },
	{ re: "SGN:SKP:ECHO CMD=(\\d+),TYPE=(\\d+)", d: "ECHO messages do not need to be signed" },
	{ re: "SGN:NCE:LEFT=(\\d+)", d: "<b>$1</b> number of nonce requests between successful verifications left before node is locked" },
	{ re: "SGN:NCE:XMT,TO=(\\d+)", d: "Nonce data transmitted to node <b>$1</b>" },
	{ re: "!SGN:NCE:XMT,TO=(\\d+) FAIL", d: "Nonce data not properly transmitted to node <b>$1</b>" },
	{ re: "!SGN:NCE:GEN", d: "Failed to generate nonce" },
	{ re: "SGN:NCE:NSUP (DROPPED)", d: "Ignored nonce/request for nonce (signing not supported)" },
	{ re: "SGN:NCE:FROM=(\\d+)", d: "Received nonce from node <b>$1</b>" },
	{ re: "SGN:NCE:(\\d+)!=(\\d+) (DROPPED)", d: "Ignoring nonce as it did not come from the desgination of the message to sign" },
	{ re: "!SGN:BND:INIT FAIL", d: "Failed to initialize signing backend" },
	{ re: "!SGN:BND:PWD<8", d: "Signing password too short" },
	{ re: "!SGN:BND:PER", d: "Backend not personalized" },
	{ re: "!SGN:BND:SER", d: "Could not get device unique serial from backend" },
	{ re: "!SGN:BND:TMR", d: "Backend timed out" },
	{ re: "!SGN:BND:SIG,SIZE,(\\d+)>(\\d+)", d: "Refusing to sign message with length <b>$1</b> because it is bigger than allowed size <b>$2</b> " },
	{ re: "SGN:BND:SIG WHI,ID=(\\d+)", d: "Salting message with our id <b>$1</b>" },
	{ re: "SGN:BND:SIG WHI,SERIAL=(.*)", d: "Salting message with our serial <b>$1</b>" },
	{ re: "!SGN:BND:VER ONGOING", d: "Verification failed, no ongoing session" },
	{ re: "!SGN:BND:VER,IDENT=(\\d+)", d: "Verification failed, identifier <b>$1</b> is unknown" },
	{ re: "SGN:BND:VER WHI,ID=(\\d+)", d: "Id <b>$1</b> found in whitelist" },
	{ re: "SGN:BND:VER WHI,SERIAL=(.*)", d: "Expecting serial <b>$1</b> for this sender" },
	{ re: "!SGN:BND:VER WHI,ID=(\\d+) MISSING", d: "Id <b>$1</b> not found in whitelist" },
	{ re: "SGN:BND:NONCE=(.*)", d: "Calculating signature using nonce <b>$1</b>" },
	{ re: "SGN:BND:HMAC=(.*)", d: "Calculated signature is <b>$1</b>" },

	// NodeManager

	{ re: "NM:INIT:VER=(.*)", d: "NodeManager version <b>$1</b>" },
	{ re: "NM:INIT:INO=(.*)", d: "Sketch <b>$1</b>" },
	{ re: "NM:INIT:LIB VER=(.+) CP=(.+)", d: "MySensors Library version <b>$1</b>, capabilities <b>$2</b>" },
	{ re: "NM:INIT:RBT p=(\\d+)", d: "Configured reboot pin <b>$1</b>" },
	{ re: "NM:BFR:INIT", d: "Connecting to the gateway..." },
	{ re: "NM:BFR:OK", d: "Connection to the gateway successful" },
	{ re: "NM:BFR:INT p=(\\d+) m=(\\d+)", d: "Setting up interrupt on pin <b>$1</b> with mode <b>$2</b>" },
	{ re: "NM:PRES:(\\w+)\\((\\d+)\\) p=(\\d+) t=(\\d+)", d: "Presented to the gateway <b>child $2</b> for sensor <b>$1</b> as <b>{type:0:$3}</b> with type <b>{type:1:$4}</b>" },
	{ re: "NM:STP:ID=(\\d+) M=(\\d+)", d: "This node has id <b>$1</b> and metric is set to <b>$2</b>" },
	{ re: "NM:STP:SD T=(\\d+)", d: "Connected SD card reader, type <b>$1</b>" },
	{ re: "NM:STP:HW V=(\\d+) F=(\\d+) M=(\\d+)", d: "CPU Vcc is <b>$1 mV</b>, CPU frequency <b>$2 Mhz</b>, free memory <b>$3 bytes</b>" },
	{ re: "NM:LOOP:(\\w+)\\((\\d+)\\):SET t=(\\d+) v=(.+)", d: "New value for <b>child $2</b> of sensor <b>$1</b> with <b>{type:1:$3}</b> = <b>$4</b>" },
	{ re: "NM:LOOP:INT p=(\\d+) v=(\\d+)", d: "Interrupt received on pin <b>$1</b>, value <b>$2</b>" },
	{ re: "NM:LOOP:INPUT\\.\\.\\.", d: "Waiting for input from the serial port" },
	{ re: "NM:LOOP:INPUT v=(.*)", d: "Received an input from the serial port: <b>$1</b>" },
	{ re: "NM:TIME:REQ", d: "Requesting the time to the controller" },
	{ re: "NM:TIME:OK ts=(\\d+)", d: "Received the time from the controller: <b>$1</b>" },
	{ re: "NM:SLP:WKP", d: "Wakeup requested" },
	{ re: "NM:SLP:SLEEP s=(\\d+)", d: "Going to sleep for <b>$1</b> seconds" },
	{ re: "NM:SLP:AWAKE", d: "Waking up from sleep" },
	{ re: "NM:SLP:LOAD s=(\\d+)", d: "Loaded configured sleep time: <b>$1</b> seconds" },
	{ re: "NM:MSG:SEND\\((\\d+)\\) t=(\\d+) p=(.+)", d: "<b>Child $1</b> sent <b>{type:1:$2}</b> = <b>$3</b>" },
	{ re: "NM:MSG:RECV\\((\\d+)\\) c=(\\d+) t=(\\d+) p=(.+)", d: "Received a <b>{command:$2}</b> message for <b>child $1</b> with <b>{type:$2:$3}</b> = <b>$4</b>" },
	{ re: "NM:PWR:RBT", d: "Rebooting the node as requested" },
	{ re: "NM:PWR:ON p=(\\d+)", d: "Powering <b>on</b> the sensor(s) through pin <b>$1</b>" },
	{ re: "NM:PWR:OFF p=(\\d+)", d: "Powering <b>off</b> the sensor(s) through pin <b>$1</b>" },
	{ re: "NM:OTA:REQ f=(\\d+) v=(\\d+)", d: "Over-the-air configuration change requested, function <b>$1</b> value <b>$2</b>" },
	{ re: "NM:EEPR:CLR", d: "Clearing the EEPROM as requested" },
	{ re: "NM:EEPR:LOAD i=(\\d+) v=(\\d+)", d: "Read from EEPROM at position <b>$1</b> the value <b>$2</b>" },
	{ re: "NM:EEPR:SAVE i=(\\d+) v=(\\d+)", d: "Wrote to EEPROM at position <b>$1</b> the value <b>$2</b>" },
	{ re: "NM:EEPR:(\\w+)\\((\\d+)\\):LOAD", d: "Restoring from EEPROM the value of <b>child $2</b> of sensor <b>$1</b>" },
	{ re: "NM:EEPR:(\\w+)\\((\\d+)\\):SAVE", d: "Saving to EEPROM the value of <b>child $2</b> of sensor <b>$1</b" },
	{ re: "NM:SENS:([^:]+):(.+)", d: "Sensor <b>$1</b>: $2" },
	{ re: "!NM:SENS:([^:]+):(.+)", d: "Error in sensor <b>$1</b>: <b style='color:red'>$2</b>" },
];



// Init regexes
for (var i=0, len=match.length;i<len; i++) {
	match[i].re = new RegExp("^" + match[i].re);
}

var stripPrefix = new RegExp("^" + rprefix + "(.*)");

function getQueryVariable(variable)
{
	var query = window.location.search.substring(1);
	var vars = query.split("&");
	for (var i=0;i<vars.length;i++) {
		var pair = vars[i].split("=");
		if(pair[0] == variable){return pair[1];}
	}
	return(false);
}

var splitWithTail = function(value, separator, limit) {
	var pattern, startIndex, m, parts = [];

	if(!limit) {
		return value.split(separator);
	}

	if(separator instanceof RegExp) {
		pattern = new RegExp(separator.source, 'g' + (separator.ignoreCase ? 'i' : '') + (separator.multiline ? 'm' : ''));
	} else {
		pattern = new RegExp(separator.replace(/([.*+?^${}()|\[\]\/\\])/g, '\\$1'), 'g');
	}

	do {
		startIndex = pattern.lastIndex;
		if(m = pattern.exec(value)) {
			parts.push(value.substr(startIndex, m.index - startIndex));
		}
	} while(m && parts.length < limit - 1);
	parts.push(value.substr(pattern.lastIndex));

	return parts;
}


new Vue({
	el: "#parser",
	data: function() {
		return {
			source: decodeURIComponent(getQueryVariable("log") || ""),
			parsed: []
		};
	},
	mounted: function() {
		this.parse();
	},
	watch: {
		source: function() {
			this.parse();
		}
	},
	methods: {
		selector: function(cmd) {
			switch (cmd) {
				case "0":
					return "presentation";
					break;
				case "1":
				case "2":
					return "subtype";
					break;
				case "3":
					return "internal";
					break;
				case "4":
					return "stream";
					break;
			}
		},
		type: function(cmd, type) {
			var t = types[this.selector(cmd)]
			return t !== undefined ? t[type] || "Undefined" : "Undefined";
		},
		decodeLog: function(msg) {
			var self = this;
			var found = false;

			filtered = msg.match(prefilter);
 			if (filtered) {
 				msg = filtered.groups.result;
 			} else {
 				return;
 			}
			for (var i=0, len=match.length;!found &&  i<len; i++) {
				var r = match[i];
				if (r.re.test(msg)) {
					msg = msg.replace(r.re, r.d);
					// lookup and replace numerical constants with their symbols
					msg = msg.replace(/{command:(\d+)}/g, function(match, m1) { return types.command[m1] });
					msg = msg.replace(/{pt:(\d+)}/g, function(match, m1) { return types.payloadtype[m1] });
					msg = msg.replace(/{type:(\d+):(\d+)}/g, function(match, cmd, type) { return self.type(cmd, type) });
					return msg;
				}
			}
		},
		parse: function() {
			console.log(this.source);
			var self = this;
			var rows = this.source.split("\n");
			this.parsed = _.map(rows, function(r) {
				// attempt decode of serial protocol (e.g. `12;6;0;0;3;My Light\n`)
				var p = splitWithTail(r, ";", 6);
				if (p.length !== 6) {
					// probably not serial protocol, maybe debug log?
					var desc = self.decodeLog(r);
					return ["","","","",desc?"":"Unknown", r,  desc];
				}
				var sel = self.selector(p[2]);
				var desc = "";
				if (p[2] == "3" && p[4] == "9") {
					desc = self.decodeLog(p[5]);

				}
				var node = stripPrefix.exec(p[0]);
				return  [
					node[1],
					p[1],
					types.command[p[2]],
					p[3]=="1"?"true":"false",
					types[sel][p[4]],
					p[5],
					desc
				];
			});
		},
		copy: function() {
			copyTextToClipboard('https://www.mysensors.org/build/parser?log='+encodeURIComponent(this.source));
		}
	}
});
