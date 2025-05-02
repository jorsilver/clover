/*
 * Copyright (C) 2020 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

import {params} from './ros.js';

// Adjust indentation
Blockly.Python.INDENT = '    ';

//Code Gen Functions
export function generateUserCode(workspace) {
	rosDefinitions = {};
	Blockly.Python.STATEMENT_PREFIX = null;
	return Blockly.Python.workspaceToCode(workspace);
}

export function generateCode(workspace) {
	rosDefinitions = {};
	Blockly.Python.STATEMENT_PREFIX = '_b(%1)\n';
	var code = Blockly.Python.workspaceToCode(workspace);
	return code;
}


// If any new block imports any library, add that library name here.
Blockly.Python.addReservedWords('_b,_print');
Blockly.Python.addReservedWords('rospy,cv2,srv,Trigger,get_telemetry,navigate,set_velocity,set_postion,land');
Blockly.Python.addReservedWords('navigate_wait,land_wait,wait_arrival,wait_yaw,get_distance');
Blockly.Python.addReservedWords('Image,CameraInfo,PointStamped,Point,CvBridge,long_callback,pyzbar')
Blockly.Python.addReservedWords('numpy,tf2_ros,tf2_geometry_msgs,image_geometry')
Blockly.Python.addReservedWords('pigpio,pi,Range');
Blockly.Python.addReservedWords('SetLEDEffect,set_effect,led_count,get_led_count');
Blockly.Python.addReservedWords('SetLEDs,LEDState,set_leds');

//ROS Service Trigger
const IMPORT_SRV = `from clover import srv
from std_srvs.srv import Trigger`;

const importMath = () => Blockly.Python.definitions_['import_math'] = 'import math';

//ROS Func Defs
const NAVIGATE_WAIT = () => `\ndef navigate_wait(x=0, y=0, z=0, speed=0.5, frame_id='body', auto_arm=False):
    res = navigate(x=x, y=y, z=z, yaw=float('nan'), speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        raise Exception(res.message)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}:
            return
        rospy.sleep(${params.sleep_time})\n`;

const NAVIGATE_GLOBAL_WAIT = () => `\ndef navigate_global_wait(lat, lon, z, speed=0.5):
    res = navigate_global(lat=lat, lon=lon, z=z, yaw=float('inf'), speed=speed)

    if not res.success:
        raise Exception(res.message)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_global_tolerance}:
            return
        rospy.sleep(${params.sleep_time})\n`;

const LAND_WAIT = () => `\ndef land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(${params.sleep_time})\n`;

const WAIT_YAW = () => `\ndef wait_yaw():
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if abs(telem.yaw) < math.radians(${params.yaw_tolerance}):
            return
        rospy.sleep(${params.sleep_time})\n`;

const WAIT_ARRIVAL = () => `\ndef wait_arrival():
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}:
            return
        rospy.sleep(${params.sleep_time})\n`;

const WAIT_ARRIVAL_PARAMS = () => `\ndef wait_arrival(tolerance=0.1, time=1.0, telem=get_telemetry(frame_id='navigate_target')):
    while not (telem.x ** 2 + telem.y ** 2 + telem.z ** 2) ** 0.5 < tolerance:
       telem = get_telemetry(frame_id='navigate_target')
    rospy.sleep(time)\n`;

const IMG_XY_TO_POINT = () => `\ncamera_info = rospy.wait_for_message('main_camera/camera_info', CameraInfo)`
	+ `\ncamera_matrix = np.float64(camera_info.K).reshape(3, 3)`
	+ `\ndistortion = np.float64(camera_info.D).flatten()\ndef img_xy_to_point(xy, dist):
	xy = cv2.undistortPoints(xy, camera_matrix, distortion, P=camera_matrix)[0][0]
	xy -= camera_info.width // 2, camera_info.height // 2
	fx = camera_matrix[0, 0]
	fy = camera_matrix[1, 1]
	return Point(x=xy[0] * dist / fx, y=xy[1] * dist / fy, z=dist)\n`;

const GET_CENTER_OF_MASS = () => `\ndef get_center_of_mass(mask):
    M = cv2.moments(mask)
    return (M['m10'] // M['m00'], M['m01'] // M['m00']) if M['m00'] != 0 else None\n`;

const IMAGE_CALLBACK = () => `\npoint_pub = rospy.Publisher('~red_circle', PointStamped, queue_size=1)`
	+ `\nfound = False\n@long_callback\ndef image_callback(msg):
    img_hsv = cv2.cvtColor(bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(img_hsv, (0, 150, 150), (15, 255, 255))
    mask2 = cv2.inRange(img_hsv, (160, 150, 150), (180, 255, 255))
    mask = cv2.bitwise_or(mask1, mask2)

    global found
    xy = get_center_of_mass(mask)
    if not found and xy is None: 
        move_to_next_search_point()
        return
    
    found = True
    altitude = get_telemetry('terrain').z
    point_pub.publish(PointStamped(msg.header, img_xy_to_point(xy, altitude)))\n`;

const TARGET_CALLBACK = () => `\ntf_buffer = tf2_ros.Buffer()\ntf_listener = tf2_ros.TransformListener(tf_buffer)`
	+ `\nseconds = None\ndef target_callback(msg):
    global seconds
    seconds = seconds or rospy.get_time()
    setpoint = tf_buffer.transform(msg, 'map', timeout=rospy.Duration(0.2))

    if rospy.get_time() - seconds < 5:
        return set_position(x=setpoint.point.x, y=setpoint.point.y, z=float('nan'), yaw=float('nan'), frame_id=setpoint.header.frame_id)

    land()
    rospy.signal_shutdown('Centered on Red Circle - Landing')\n`;

const ARRIVED = () => `\ndef arrived():
    telem = get_telemetry(frame_id='navigate_target')
    return math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < ${params.navigate_tolerance}\n`

const GET_DISTANCE = `\ndef get_distance(x, y, z, frame_id):
    telem = get_telemetry(frame_id)
    return math.sqrt((x - telem.x) ** 2 + (y - telem.y) ** 2 + (z - telem.z) ** 2)\n`;

var rosDefinitions = {};


//Add ROS Service Proxies, and ROS Func Defs
function generateROSDefinitions() {
	// order for ROS definitions is significant, so generate all ROS definitions as one
	var code = rosDefinitions.cv ?
		`rospy.init_node('cv', disable_signals=True)\nbridge = CvBridge()\n\n` 
		: `rospy.init_node('flight')\n\n`;

	const rosServiceProxy = (name, srvType, proxyDir = '', persistent = false) => 
        code += `${name} = rospy.ServiceProxy('${proxyDir}${name}', ${srvType}${persistent ? ', persistent=True' : ''})\n`;

	if (rosDefinitions.offboard) {
        rosServiceProxy('get_telemetry', 'srv.GetTelemetry');
		rosServiceProxy('navigate', 'srv.Navigate');
		rosDefinitions.cv && rosServiceProxy('set_position', 'srv.SetPosition')
        rosDefinitions.navigateGlobal && rosServiceProxy('navigate_global', 'srv.NavigateGlobal');
        rosDefinitions.setYaw && rosServiceProxy('set_yaw', 'srv.SetYaw');
        rosDefinitions.setVelocity && rosServiceProxy('set_velocity', 'srv.SetVelocity');
        rosDefinitions.setAttitude && rosServiceProxy('set_attitude', 'srv.SetAttitude');
        rosDefinitions.setRates && rosServiceProxy('set_rates', 'srv.SetRates');
        rosServiceProxy('land', 'Trigger');
    }

	rosDefinitions.setEffect && rosServiceProxy('set_effect', 'SetLEDEffect', 'led/', true);
    rosDefinitions.setLeds && rosServiceProxy('set_leds', 'SetLEDs', 'led/', true);

	if (rosDefinitions.cv) {
		code += WAIT_ARRIVAL_PARAMS();
		code += IMG_XY_TO_POINT();
		code += GET_CENTER_OF_MASS();
		code += IMAGE_CALLBACK();
		code += TARGET_CALLBACK();
	}

    rosDefinitions.landWait && (code += LAND_WAIT());
    rosDefinitions.navigateWait && (code += NAVIGATE_WAIT());
    rosDefinitions.navigateGlobalWait && (code += NAVIGATE_GLOBAL_WAIT());
    rosDefinitions.waitArrival && (code += WAIT_ARRIVAL());
    rosDefinitions.arrived && (code += ARRIVED());
    rosDefinitions.waitYaw && (code += WAIT_YAW());
    rosDefinitions.distance && (code += GET_DISTANCE);

	Blockly.Python.definitions_['ros'] = code;
}


//Generator Helper Funcs
function initNode() {
	Blockly.Python.definitions_['import_rospy'] = 'import rospy';
	generateROSDefinitions();
}

function simpleOffboard() {
	rosDefinitions.offboard = true;
	let import_srv = rosDefinitions.cv ?
		IMPORT_SRV.replace(" srv", " long_callback, srv") : IMPORT_SRV;
	Blockly.Python.definitions_['import_srv'] = import_srv;
	initNode();
}

function buildFrameId(block) {
	let frame = block.getFieldValue('FRAME_ID').toLowerCase();
	let id = Blockly.Python.valueToCode(block, 'ID', Blockly.Python.ORDER_NONE);
	if (frame == 'aruco') { // aruco marker frame
		if (id.match(/^[0-9]+$/)) { // id is positive integer
			return `'${frame}_${id}'`;
		} else { // something else...
			return `'${frame}_' + str(int(${id}))`;
		}
	} else {
		return `'${frame}'`;
	}
}


//Generators (FLIGHT)
Blockly.Python.take_off = function(block) {
	simpleOffboard();

	let z = Blockly.Python.valueToCode(block, 'ALT', Blockly.Python.ORDER_NONE);

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.navigateWait = true;
		importMath();
		simpleOffboard();

		return `navigate_wait(z=${z}, frame_id='body', auto_arm=True)\n`;
	} else {
		return `navigate(z=${z}, frame_id='body', auto_arm=True)\n`;
	}
}

Blockly.Python.navigate = function(block) {
	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let lat = Blockly.Python.valueToCode(block, 'LAT', Blockly.Python.ORDER_NONE);
	let lon = Blockly.Python.valueToCode(block, 'LON', Blockly.Python.ORDER_NONE);
	let wait = block.getFieldValue('WAIT') == 'TRUE';
	let frameId = block.getFieldValue('FRAME_ID');
	let speed = Blockly.Python.valueToCode(block, 'SPEED', Blockly.Python.ORDER_NONE);

	simpleOffboard();

	// global coordinates
	if (frameId.startsWith('GLOBAL')) {
		rosDefinitions.navigateGlobal = true;
		simpleOffboard();

		if (frameId == 'GLOBAL') {
			z = `${z} + get_telemetry().alt - get_telemetry().z`;
		}

		if (wait) {
			rosDefinitions.navigateGlobalWait = true;
			importMath();
			simpleOffboard();
			return `navigate_global_wait(lat=${lat}, lon=${lon}, z=${z}, speed=${speed})\n`;

		} else {
			return `navigate_global(lat=${lat}, lon=${lon}, z=${z}, yaw=float('inf'), speed=${speed})\n`;
		}

	} else {
		frameId = buildFrameId(block);
		let params = [`x=${x}`, `y=${y}`, `z=${z}`, `frame_id=${frameId}`, `speed=${speed}`];

		if (wait) {
			rosDefinitions.navigateWait = true;
			importMath();
			simpleOffboard();

			return `navigate_wait(${params.join(', ')})\n`;

		} else {
			if (frameId != 'body') {
				params.push(`yaw=float('nan')`);
			}
			return `navigate(${params.join(', ')})\n`;
		}
	}
}

Blockly.Python.set_velocity = function(block) {
	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);

	simpleOffboard();

	if (frameId == `'body'`) {
		return `set_velocity(vx=${x}, vy=${y}, vz=${z}, frame_id=${frameId})\n`;
	} else {
		return `set_velocity(vx=${x}, vy=${y}, vz=${z}, yaw=float('nan'), frame_id=${frameId})\n`;
	}
}

Blockly.Python.setpoint = function(block) {
	var type = block.getFieldValue('TYPE');
	let frameId = buildFrameId(block);
	let vx = Blockly.Python.valueToCode(block, 'VX', Blockly.Python.ORDER_NONE);
	let vy = Blockly.Python.valueToCode(block, 'VY', Blockly.Python.ORDER_NONE);
	let vz = Blockly.Python.valueToCode(block, 'VZ', Blockly.Python.ORDER_NONE);
	let yaw = Blockly.Python.valueToCode(block, 'YAW', Blockly.Python.ORDER_NONE);
	let pitch = Blockly.Python.valueToCode(block, 'PITCH', Blockly.Python.ORDER_NONE);
	let roll = Blockly.Python.valueToCode(block, 'ROLL', Blockly.Python.ORDER_NONE);
	let thrust = Blockly.Python.valueToCode(block, 'THRUST', Blockly.Python.ORDER_NONE);

	if (type == 'VELOCITY') {
		rosDefinitions.setVelocity = true;
		simpleOffboard();
		return `set_velocity(vx=${vx}, vy=${vy}, vz=${vz}, frame_id=${frameId}, yaw=float('nan'))\n`;
	} else if (type == 'ATTITUDE') {
		rosDefinitions.setAttitude = true;
		simpleOffboard();
		return `set_attitude(roll=${roll}, pitch=${pitch}, yaw=${yaw}, thrust=${thrust}, frame_id=${frameId})\n`;
	} else if (type == 'RATES') {
		rosDefinitions.setRates = true;
		simpleOffboard();
		return `set_rates(roll_rate=${roll}, pitch_rate=${pitch}, yaw_rate=${yaw}, thrust=${thrust})\n`;
	}
}

Blockly.Python.land = function(block) {
	simpleOffboard();

	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.landWait = true;
		simpleOffboard();

		return `land_wait()\n`;
	} else {
		return 'land()\n';
	}
}

Blockly.Python.set_yaw = function(block) {
	rosDefinitions.setYaw = true;
	simpleOffboard();
	let yaw = Blockly.Python.valueToCode(block, 'YAW', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);
	let code = `set_yaw(yaw=${yaw}, frame_id=${frameId})\n`;
	if (block.getFieldValue('WAIT') == 'TRUE') {
		rosDefinitions.waitYaw = true;
		importMath();
		simpleOffboard();
		code += 'wait_yaw()\n';
	}
	return code;
}

Blockly.Python.wait_arrival = function(block) {
	rosDefinitions.waitArrival = true;
	importMath();
	simpleOffboard();
	return 'wait_arrival()\n';
}

Blockly.Python.wait = function(block) {
	initNode();
	return `rospy.sleep(${Blockly.Python.valueToCode(block, 'TIME', Blockly.Python.ORDER_NONE)})\n`;
}


//Generators (STATE)
Blockly.Python.get_time = function(block) {
	initNode();
	return ['rospy.get_time()', Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.arrived = function(block) {
	rosDefinitions.arrived = true;
	importMath();
	simpleOffboard();
	return ['arrived()', Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_position = function(block) {
	simpleOffboard();
	let frameId = buildFrameId(block);
	var code = `get_telemetry(${frameId}).${block.getFieldValue('FIELD').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.angle = function(block) {
	importMath();
	return [`math.radians(${block.getFieldValue('ANGLE')})`, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_yaw = function(block) {
	importMath();
	simpleOffboard();
	let frameId = buildFrameId(block);
	var code = `math.degrees(get_telemetry(${frameId}).yaw)`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_attitude = function(block) {
	importMath();
	simpleOffboard();
	var code = `math.degrees(get_telemetry().${block.getFieldValue('FIELD').toLowerCase()})`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.global_position = function(block) {
	simpleOffboard();
	var code = `get_telemetry().${block.getFieldValue('FIELD').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.distance = function(block) {
	rosDefinitions.distance = true;
	importMath();
	simpleOffboard();

	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let z = Blockly.Python.valueToCode(block, 'Z', Blockly.Python.ORDER_NONE);
	let frameId = buildFrameId(block);

	return [`get_distance(${x}, ${y}, ${z}, ${frameId})`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.rangefinder_distance = function(block) {
	initNode();
	Blockly.Python.definitions_['import_range'] = 'from sensor_msgs.msg import Range';
	return [`rospy.wait_for_message('rangefinder/range', Range).range`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.mode = function(block) {
	simpleOffboard();
	return [`get_telemetry().mode`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.armed = function(block) {
	simpleOffboard();
	return [`get_telemetry().armed`, Blockly.Python.ORDER_FUNCTION_CALL]
}

Blockly.Python.voltage = function(block) {
	simpleOffboard();
	var code = `get_telemetry().${block.getFieldValue('TYPE').toLowerCase()}`;
	return [code, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.get_rc = function(block) {
	Blockly.Python.definitions_['import_rcin'] = 'from mavros_msgs.msg import RCIn';
	var channel = Blockly.Python.valueToCode(block, 'CHANNEL', Blockly.Python.ORDER_NONE);
	return [`rospy.wait_for_message('mavros/rc/in', RCIn).channels[${channel}]`, Blockly.Python.ORDER_FUNCTION_CALL]
}


/* LED Category */

//Func Defs (LED)
const PARSE_COLOR = `def ${Blockly.Python.FUNCTION_NAME_PLACEHOLDER_}(color):
	return {'r': int(color[1:3], 16), 'g': int(color[3:5], 16), 'b': int(color[5:7], 16)}`;

const GET_LED_COUNT = `led_count = None
	def get_led_count():
		global led_count
		if led_count is None:
			led_count = len(rospy.wait_for_message('led/state', LEDStateArray, timeout=10).leds)
		return led_count\n`;

	
//Generator Helper Funcs (LED)
function parseColor(color) {
	return {
		r: parseInt(color.substr(2, 2), 16),
		g: parseInt(color.substr(4, 2), 16),
		b: parseInt(color.substr(6, 2), 16)
	}
}


//Generators (LED)
Blockly.Python.set_effect = function(block) {// TODO: weird code with colour_rgb block
	rosDefinitions.setEffect = true;
	Blockly.Python.definitions_['import_led_effect'] = 'from clover.srv import SetLEDEffect';
	initNode();

	var effect = block.getFieldValue('EFFECT').toLowerCase();

	if (effect == 'rainbow' || effect == 'rainbow_fill') {
		return `set_effect(effect='${effect}')\n`;
	} else {
		let colorCode = Blockly.Python.valueToCode(block, 'COLOR', Blockly.Python.ORDER_NONE);

		if (/^'(.*)'$/.test(colorCode)) { // is simple string
			let color = parseColor(colorCode);
			return `set_effect(effect='${effect}', r=${color.r}, g=${color.g}, b=${color.b})\n`;
		} else {
			let parseColor = Blockly.Python.provideFunction_('parse_color', [PARSE_COLOR]);
			return `set_effect(effect='${effect}', **${parseColor}(${colorCode}))\n`;
		}
	}
}

Blockly.Python.set_led = function(block) {
	rosDefinitions.setLeds = true;
	Blockly.Python.definitions_['import_set_led'] = 'from led_msgs.srv import SetLEDs\nfrom led_msgs.msg import LEDState';
	initNode();

	var index = Blockly.Python.valueToCode(block, 'INDEX', Blockly.Python.ORDER_NONE);
	var colorCode = Blockly.Python.valueToCode(block, 'COLOR', Blockly.Python.ORDER_NONE);

	if (/^'(.*)'$/.test(colorCode)) { // is simple string
		let color = parseColor(colorCode);
		return `set_leds([LEDState(index=int(${index}), r=${color.r}, g=${color.g}, b=${color.b})])\n`; // TODO: check for simple int
	} else {
		let parseColor = Blockly.Python.provideFunction_('parse_color', [PARSE_COLOR]);
		return `set_leds([LEDState(index=${index}, **${parseColor}(${colorCode}))])\n`;
	}
}

Blockly.Python.led_count = function(block) {
	initNode();
	Blockly.Python.definitions_['import_led_state_array'] = 'from led_msgs.msg import LEDStateArray';
	Blockly.Python.definitions_['get_led_count'] = GET_LED_COUNT;
	return [`get_led_count()`, Blockly.Python.ORDER_FUNCTION_CALL]
}


/* GPIO Category */

//Func Defs (GPIO)
const GPIO_READ = `\ndef gpio_read(pin):
    pi.set_mode(pin, pigpio.INPUT)
    return pi.read(pin)\n`;

const GPIO_WRITE = `\ndef gpio_write(pin, level):
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, level)\n`;

const SET_SERVO = `\ndef set_servo(pin, pwm):
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_servo_pulsewidth(pin, pwm)\n`;

const SET_DUTY_CYCLE = `\ndef set_duty_cycle(pin, duty_cycle):
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.set_PWM_dutycycle(pin, duty_cycle * 255)\n`;


//Generator Helper Funcs (GPIO)
function pigpio() {
	Blockly.Python.definitions_['import_pigpio'] = 'import pigpio';
	Blockly.Python.definitions_['init_pigpio'] = 'pi = pigpio.pi()\nif not pi.connected: raise Exception(\'Cannot connect to pigpiod\')';
}


//Generators (GPIO)
Blockly.Python.gpio_read = function(block) {
	pigpio();
	Blockly.Python.definitions_['gpio_read'] = GPIO_READ;
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	return [`gpio_read(${pin})`, Blockly.Python.ORDER_FUNCTION_CALL];
}

Blockly.Python.gpio_write = function(block) {
	pigpio();
	Blockly.Python.definitions_['gpio_write'] = GPIO_WRITE;
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	var level = Blockly.Python.valueToCode(block, 'LEVEL', Blockly.Python.ORDER_NONE);
	return `gpio_write(${pin}, ${level})\n`;
}

Blockly.Python.set_servo = function(block) {
	pigpio();
	Blockly.Python.definitions_['set_servo'] = SET_SERVO;
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	var pwm = Blockly.Python.valueToCode(block, 'PWM', Blockly.Python.ORDER_NONE);
	return `set_servo(${pin}, ${pwm})\n`;
}

Blockly.Python.set_duty_cycle = function(block) {
	pigpio();
	Blockly.Python.definitions_['set_duty_cycle'] = SET_DUTY_CYCLE;
	var pin = Blockly.Python.valueToCode(block, 'PIN', Blockly.Python.ORDER_NONE);
	var dutyCycle = Blockly.Python.valueToCode(block, 'DUTY_CYCLE', Blockly.Python.ORDER_NONE);
	return `set_duty_cycle(${pin}, ${dutyCycle})\n`;
}

function cv () {
	Blockly.Python.definitions_['import_cv2'] = 'import cv2';
	Blockly.Python.definitions_['import_image'] = 'from sensor_msgs.msg import Image, CameraInfo';
	Blockly.Python.definitions_['import_point_stamped'] = 'from geometry_msgs.msg import PointStamped, Point';
	Blockly.Python.definitions_['import_cv_bridge'] = 'from cv_bridge import CvBridge';
}

function search_grid (x, y, size) {
	let code = `\nx_range = np.round(np.linspace(${x} - ${size} / 2, ${x} + ${size} / 2, int(${size}/.33) + 1),2)`;
	code += `\ny_range = np.round(np.linspace(${y} - ${size} / 2, ${y} + ${size} / 2, int(${size}/1) + 1),2)`;
	code += `\nsearch_pattern = ((x, y) for i, y in enumerate(y_range)
    for x in (x_range if i % 2 == 0 else reversed(x_range)))`;
	return code;
}

const NEXT_SEARCH_POINT = () => `\norigin = get_telemetry()\ndef move_to_next_search_point():
    try:
        cur_target = next(search_pattern)
        set_position(x=cur_target[0], y=cur_target[1], z=float('nan'), yaw=float('nan'), frame_id='map')
        wait_arrival(tolerance=0.66, time=-1)
    except StopIteration:
        navigate(x=origin.x, y=origin.y, z=0.5, yaw=float('nan'), speed=1, frame_id='map')
        wait_arrival(time=2)
        land()
        rospy.signal_shutdown("Circle not found.")`;


Blockly.Python.find_target = function(block) {
	cv();
	Blockly.Python.definitions_['import_numpy'] = 'import numpy as np';
	Blockly.Python.definitions_['import_tf2_ros'] = 'import tf2_ros';
	Blockly.Python.definitions_['import_tf2_geometry_msgs'] = 'import tf2_geometry_msgs';
	rosDefinitions.cv = true;
	simpleOffboard();
	let x = Blockly.Python.valueToCode(block, 'X', Blockly.Python.ORDER_NONE);
	let y = Blockly.Python.valueToCode(block, 'Y', Blockly.Python.ORDER_NONE);
	let size = Blockly.Python.valueToCode(block, 'SIZE', Blockly.Python.ORDER_NONE);
	Blockly.Python.definitions_['ros'] += search_grid(x, y, size) + NEXT_SEARCH_POINT();
	let code = `\npattern_start = next(search_pattern)`;
	code += `\nnavigate(x=pattern_start[0], y=pattern_start[1], z=float('nan'), speed=1)`;
	code += `\nwait_arrival()`;
	code += `\nimage_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)`;
	code += `\ntarget_sub = rospy.Subscriber('~red_circle', PointStamped, target_callback, queue_size=1)`;
	code += `\nrospy.spin()`;
	return code;
}