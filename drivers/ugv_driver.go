package drivers

import (
	"encoding/json"
	"fmt"
	"log"
	"strings"
	"sync"
	"time"

	"go.bug.st/serial"
)

/* UGVDriver is a driver for the UGV control board. */
type UGVDriver struct {
	Device string
	Mode   serial.Mode
}

var mu sync.Mutex

var port serial.Port

const readIntervalMs = 50

/* Initialise the UGV control board. */
func (driver *UGVDriver) Init() error {
	var err error
	port, err = serial.Open(driver.Device, &driver.Mode)
	if err != nil {
		log.Println("Error while opening serial port:", err)
		return err
	}

	// driver.SetFeedbackInterval(readIntervalMs * 2)               // set feedback interval
	driver.EnableFeedback(false)                                    // serial feedback flow off
	driver.EnableEchoMode(false)                                    // serial echo off
	driver.SetModuleType(0)                                         // select the module - 0:None, 1:RoArm-M2-S, 2:Gimbal
	driver.SendJSON(`{"T":300,"mode":0,"mac":"EF:EF:EF:EF:EF:EF"}`) // the base won't be ctrl by esp-now broadcast cmd, but it can still recv broadcast megs.
	driver.SendJSON(`{"T":900,"main":2,"module":0}`)                // set product version
	return nil
}

func (driver *UGVDriver) Close() error {
	if port == nil {
		return nil
	}
	err := port.Close()
	if err != nil {
		return err
	}
	return nil
}

/* Send command to the UGV control board in a JSON format. */
func (driver *UGVDriver) SendJSON(cmd string) error {
	return driver.sendCommand([]byte(cmd))
}

/* Set the target linear velocities for the left and right wheels in m/s. */
func (driver *UGVDriver) SetSpeed(left, right float32) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":1,"L":%.2f,"R":%.2f}`, left, right))
}

/*
Set the Power going to the wheels.

left and right must be between -255 and 255, if out of bounds they are set to bound.
*/
func (driver *UGVDriver) SetPowerToWheels(left, right int16) error {
	left = limit(left, -255, 255)
	right = limit(right, -255, 255)

	return driver.SendJSON(fmt.Sprintf(`{"T":11,"L":%d,"R":%d}`, left, right))
}

/*
Set ROS control (velocity closed-loop control).

This command is for ROS-based host computer control of the chassis movement. x represents the linear velocity in m/s, which can be negative; z represents the angular velocity in rad/s, which can also be negative.
*/
func (driver *UGVDriver) SendROSControls(x, z float32) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":13,"X":%.2f,"Z":%.2f}`, x, z))
}

/* Set PID controller settings to default values. */
func (driver *UGVDriver) SetDefaultMotorPID() error {
	return driver.SendJSON(`{"T":2,"P":20,"I":2000,"D":0,"L":255}`)
}

/*
Set PID controller settings.

This command is used to tune the PID controller.
*/
func (driver *UGVDriver) SetMotorPID(p, i, d uint16) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":2,"P":%d,"I":%d,"D":%d,"L":255}`, p, i, d))
}

/*
Sets the text on the OLED display

lineNum is the line number. A single JSON command can change the content of one line. Upon receiving a new command, the default OLED screen that appears at startup will disappear, replaced by your added content. For most products using a 0.91-inch OLED display, lineNum can be 0, 1, 2, or 3, totaling four lines. Text is the content you want to display on this line. If the content is too long for one line, it will automatically wrap to the next line, but this may also push off the last line of content.
*/
func (driver *UGVDriver) SetOLEDText(lineNum uint8, text string) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":3,"lineNum":%d,"Text":"%s"}`, lineNum, text))
}

/* Sets the text on the OLED display to default value. */
func (driver *UGVDriver) SetDefaultOLEDText() error {
	return driver.SendJSON(`{"T":-3}`)
}

/* Sets the module type. */
func (driver *UGVDriver) SetModuleType(t uint8) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":4,"cmd":%d}`, t))
}

/* Retrieves IMU data. */
func (driver *UGVDriver) GetIMUData() (string, error) {
	err := driver.SendJSON(`{"T":126}`)
	if err != nil {
		return "", err
	}
	time.Sleep(100 * time.Millisecond)
	return driver.read()
}

/* IMU calibration (reserved interface). */
func (driver *UGVDriver) CalibrationIMUStep() error {
	return driver.SendJSON(`{"T":127}`)
}

/* Retrieves current IMU offsets (reserved interface). */
func (driver *UGVDriver) GetIMUOffset() (string, error) {
	err := driver.SendJSON(`{"T":128}`)
	if err != nil {
		return "", err
	}
	time.Sleep(100 * time.Millisecond)
	return driver.read()
}

/* Sets the IMU offsets (reserved interface). */
func (driver *UGVDriver) SetIMUOffset(x, y, z int16) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":129,"x":%d,"y":%d,"z":%d}`, x, y, z))
}

/* Chassis information feedback. */
func (driver *UGVDriver) GetBaseFeedback() (string, error) {
	err := driver.SendJSON(`{"T":130}`)
	if err != nil {
		return "", err
	}
	time.Sleep(100 * time.Millisecond)
	return driver.read()
}

/* Enables continuous chassis information feedback. */
func (driver *UGVDriver) EnableFeedback(val bool) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":131,"cmd":%d}`, bool2int(val)))
}

/*
Sets info feedback frequency

val is in ms
*/
func (driver *UGVDriver) SetFeedbackInterval(val uint16) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":142,"cmd":%d}`, val))
}

/* Enables UART echo mode (sends back commands) */
func (driver *UGVDriver) EnableEchoMode(val bool) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":143,"cmd":%d}`, bool2int(val)))
}

/*
Set WiFi Mode at Boot.

mode value 0 turns off WiFi; 1 sets to AP mode; 2 sets to STA mode; 3 sets to AP+STA mode.
*/
func (driver *UGVDriver) SetWiFiMode(mode uint8) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":401,"cmd":%d}`, mode))
}

/* Configure SSID and Password for AP Mode (ESP32 as a Hotspot). */
func (driver *UGVDriver) SetWiFiAP(ssid, pass string) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":402,"ssid":"%s","password":"%s"}`, ssid, pass))
}

/* Configure SSID and Password for STA Mode (ESP32 connects to a known hotspot). */
func (driver *UGVDriver) SetWiFiSTA(ssid, pass string) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":403,"ssid":"%s","password":"%s"}`, ssid, pass))
}

/* Set Names and Passwords for AP and STA Modes (AP+STA Mode). */
func (driver *UGVDriver) SetWiFiAPSTA(apSsid, apPass, staSsid, staPass string) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":404,"ap_ssid":"%s","ap_password":"%s","sta_ssid":"%s","sta_password":"%s"}`, apSsid, apPass, staSsid, staPass))
}

/* Get Current WiFi Information. */
func (driver *UGVDriver) GetWiFiInfo() (string, error) {
	err := driver.SendJSON(`{"T":405}`)
	if err != nil {
		return "", err
	}
	time.Sleep(100 * time.Millisecond)
	return driver.read()
}

/* Create a New WiFi Configuration File Using Current Settings. */
func (driver *UGVDriver) SaveWiFiConfig() error {
	return driver.SendJSON(`{"T":406}`)
}

/* Create a New WiFi Configuration File Using Input Settings. */
func (driver *UGVDriver) ConfigureWiFi(mode uint8, apSsid, apPass, staSsid, staPass string) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":407,"mode":%d,"ap_ssid":"%s","ap_password":"%s","sta_ssid":"%s","sta_password":"%s"}`, mode, apSsid, apPass, staSsid, staPass))
}

/* Disconnect WiFi Connection. */
func (driver *UGVDriver) StopWiFi() error {
	return driver.SendJSON(`{"T":408}`)
}

/* 12V Switch Output Settings. (LED lights) */
func (driver *UGVDriver) SetVoltageOutput(io4, io5 uint8) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":132,"IO4":%d,"IO5":%d}`, io4, io5))
}

/*
Basic Gimbal Control.

This command is used to control the orientation of the gimbal. X represents the horizontal orientation in degrees, with positive values turning right and negative values turning left, ranging from -180 to 180 degrees. Y represents the vertical orientation in degrees, with positive values tilting up and negative values tilting down, ranging from -30 to 90 degrees. SPD stands for speed, and ACC for acceleration; when set to 0, they indicate the fastest speed/acceleration.
*/
func (driver *UGVDriver) SendGimbalBasicControls(x, y int16, spd, acc float32) error {
	x = limit(x, -180, 180)
	y = limit(y, -30, 90)
	return driver.SendJSON(fmt.Sprintf(`{"T":133,"X":%d,"Y":%d,"SPD":%.2f,"ACC":%.2f}`, x, y, spd, acc))
}

/*
Continuous Gimbal Control Command.

This command is for continuous control over the pan-tilt's orientation. X and Y function similarly to the basic control command, specifying the desired horizontal and vertical orientations, respectively. SX and SY represent the speeds for the X and Y axes, respectively.
*/
func (driver *UGVDriver) SendGimbalControls(x, y int16, sx, sy float32) error {
	x = limit(x, -180, 180)
	y = limit(y, -30, 90)
	return driver.SendJSON(fmt.Sprintf(`{"T":134,"X":%d,"Y":%d,"SX":%.2f,"SY":%.2f}`, x, y, sx, sy))
}

/*
Pan-tilt Stop Command.

This command can be used to immediately stop the pan-tilt's movement initiated by the previous commands.
*/
func (driver *UGVDriver) StopGimbal() error {
	return driver.SendJSON(`{"T":135}`)
}

/*
Pan-tilt Stabilization Feature.

Setting s to 0 turns off this feature, and setting it to 1 enables it. When enabled, the pan-tilt automatically adjusts its vertical angle using IMU data to maintain stability. The y parameter sets the target angle between the pan-tilt and the ground, allowing the camera to look up and down even when the stabilization feature is active.
*/
func (driver *UGVDriver) SetGimbalStabilization(s bool, y int16) error {
	y = limit(y, -30, 90)
	return driver.SendJSON(fmt.Sprintf(`{"T":137,"s":%d,"y":%d}`, bool2int(s), y))
}

/*
Pan-tilt UI Control.

This command is intended for pan-tilt control via a UI interface. The X value can be -1, 0, or 1, where -1 rotates left, 0 stops, and 1 rotates right. The Y value can also be -1, 0, or 1, where -1 tilts down, 0 stops, and 1 tilts up. SPD specifies the speed of the operation.
*/
func (driver *UGVDriver) SendGimbalUserControls(x, y int16, spd float32) error {
	x = limit(x, -1, 1)
	y = limit(y, -1, 1)
	return driver.SendJSON(fmt.Sprintf(`{"T":141,"X":%d,"Y":%d,"SPD":%.2f}`, x, y, spd))
}

/*
Sets the Heartbeat Function Interval.

The interval unit is milliseconds. This command sets the interval for the heartbeat function. If the sub-controller does not receive a new motion command within this time, it will automatically stop movement. This feature helps prevent continuous movement in case the host crashes.
*/
func (driver *UGVDriver) SetHeartBeatInterval(interval uint16) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":136,"cmd":%d}`, interval))
}

/*
Sets the Speed Ratio for Left and Right.

Due to potential errors in encoders or tire traction, the device might not move straight even when both wheels are set to the same speed. This command allows for fine-tuning the speed of the left and right wheels to correct this issue. For example, if the left wheel needs to rotate slower, you can change the value of left to 0.98. Try not to set the values of left and right greater than one.
*/
func (driver *UGVDriver) SetSpeedRatio(left, right float32) error {
	left = limitf(left, 0, 1)
	right = limitf(right, 0, 1)
	return driver.SendJSON(fmt.Sprintf(`{"T":138,"L":%.4f,"R":%.4f}`, left, right))
}

/* Retrieves the Current Speed Ratio settings. */
func (driver *UGVDriver) GetSpeedRatio() (string, error) {
	err := driver.SendJSON(`{"T":139}`)
	if err != nil {
		return "", err
	}
	time.Sleep(100 * time.Millisecond)
	return driver.read()
}

/* Scans the current task files. */
func (driver *UGVDriver) ScanTasks() (string, error) {
	err := driver.SendJSON(`{"T":200}`)
	if err != nil {
		return "", err
	}
	time.Sleep(100 * time.Millisecond)
	return driver.read()
}

/* Creates a new task files. */
func (driver *UGVDriver) CreateTask(name, content string) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":201,"name":"%s","content":"%s"}`, name, content))
}

/* Reads a task files. */
func (driver *UGVDriver) ReadTask(name string) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":202,"name":"%s"}`, name))
}

/* Deletes a task file. */
func (driver *UGVDriver) DeleteTask(name string) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":203,"name":"%s"}`, name))
}

/* Adds a new instruction at the end of a task file. */
func (driver *UGVDriver) AppendTaskLine(name, content string) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":204,"name":"%s","content":"%s"}`, name, content))
}

/* Insert a new instruction in the middle of a task file. */
func (driver *UGVDriver) InsertInstruction(name, content string, lineNum uint16) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":205,"name":"%s","lineNum":%d,"content":"%s"}`, name, lineNum, content))
}

/* Replaces an instruction in a task file. */
func (driver *UGVDriver) ReplaceInstruction(name, content string, lineNum uint16) error {
	return driver.SendJSON(fmt.Sprintf(`{"T":206,"name":"%s","lineNum":%d,"content":"%s"}`, name, lineNum, content))
}

/* Reboot the ESP32. */
func (driver *UGVDriver) Reboot() error {
	return driver.SendJSON(`{"T":600}`)
}

/* Retrieves the remaining space size in the FLASH memory. */
func (driver *UGVDriver) GetFreeFlashSpace() (string, error) {
	err := driver.SendJSON(`{"T":601}`)
	if err != nil {
		return "", err
	}
	time.Sleep(100 * time.Millisecond)
	return driver.read()
}

/* Outputs the current boot mission file. */
func (driver *UGVDriver) GetBootMission() (string, error) {
	err := driver.SendJSON(`{"T":602}`)
	if err != nil {
		return "", err
	}
	time.Sleep(100 * time.Millisecond)
	return driver.read()
}

/* Resets the boot mission file to its default or a predetermined state. */
func (driver *UGVDriver) ResetBootMission() error {
	return driver.SendJSON(`{"T":603}`)
}

/* Clears the ESP32's Non-Volatile Storage (NVS) area. This command can be useful if there are issues with establishing a WiFi connection. It's recommended to reboot the ESP32 after executing this command. */
func (driver *UGVDriver) ClearStorage() error {
	return driver.SendJSON(`{"T":604}`)
}

/*
Sets the mode for information feedback.

When val is set to 1, it enables the printing of debug information. Setting val to 2 enables continuous feedback of chassis information. Setting val to 0 turns off feedback, meaning no information will be provided.
*/
func (driver *UGVDriver) SetFeedbackMode() error {
	return driver.SendJSON(`{"T":605,"cmd":1}`)
}

func (driver *UGVDriver) sendCommand(cmd []byte) error {
	mu.Lock()
	_, err := port.Write(append(cmd, '\n'))
	port.Drain()
	port.ResetOutputBuffer()
	mu.Unlock()
	if err != nil {
		log.Println("Error while sending command:", err)
		return err
	}

	return nil
}

func (driver *UGVDriver) read() (string, error) {
	buf := make([]byte, 512)
	var sb strings.Builder
	sb.Grow(len(buf) * 3) // Preallocate buffer size
	attempts := 0
	for {
		// Read data from the serial port
		mu.Lock()
		n, err := port.Read(buf)
		mu.Unlock()
		if err != nil {
			log.Printf("Error reading from serial port: %v", err)
			time.Sleep(time.Millisecond * readIntervalMs) // Wait before retrying
			continue
		}

		if n > 0 {
			sb.Write(buf[:n])
			log.Println("Read n bytes:", n)
			log.Println("Read buffer:", sb.String())
			receivedData := FirstLine(sb)
			if json.Valid([]byte(receivedData)) {
				log.Printf("Read: %s\n", receivedData)
				return string(receivedData), nil
			}

			attempts++
			if attempts > 3 {
				return "", fmt.Errorf("failed to read json from serial port after 3 attempts")
			}
			time.Sleep(time.Millisecond * readIntervalMs) // Wait before retrying
			continue
		}
	}
}

func FirstLine(sb strings.Builder) string {
	str := sb.String()
	for i := 0; i < len(str); i++ {
		if str[i] == '\n' {
			return str[:i]
		}
	}
	return str
}

func bool2int(b bool) uint8 {
	if b {
		return 1
	}
	return 0
}

func limit(val, min, max int16) int16 {
	if val > max {
		return max
	}
	if val < min {
		return min
	}
	return val
}

func limitf(val, min, max float32) float32 {
	if val > max {
		return max
	}
	if val < min {
		return min
	}
	return val
}
