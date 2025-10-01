package drivers

import (
	"fmt"
	"log"
	"sync"

	"go.bug.st/serial"
)

var mu sync.Mutex

/* UGVDriver is a driver for the UGV control board. */
type UGVDriver struct {
	Device string
	Mode   serial.Mode
}

/* Initialise the UGV control board. */
func (d *UGVDriver) Init() error {
	d.sendCommand([]byte(`{"T":142,"cmd":50}`))                           // set feedback interval
	d.sendCommand([]byte(`{"T":131,"cmd":1}`))                            // serial feedback flow on
	d.sendCommand([]byte(`{"T":143,"cmd":0}`))                            // serial echo off
	d.sendCommand([]byte(`{"T":4,"cmd":0}`))                              // select the module - 0:None, 1:RoArm-M2-S, 2:Gimbal
	d.sendCommand([]byte(`{"T":300,"mode":0,"mac":"EF:EF:EF:EF:EF:EF"}`)) // the base won't be ctrl by esp-now broadcast cmd, but it can still recv broadcast megs.
	d.sendCommand([]byte(`{"T":900,"main":2,"module":0}`))                // set product version
	return nil
}

/* Send command to the UGV control board in a JSON format. */
func (d *UGVDriver) SendJSON(cmd string) error {
	return d.sendCommand([]byte(cmd))
}

/* Set the target linear velocities for the left and right wheels in m/s. */
func (d *UGVDriver) SetSpeed(left, right float32) error {
	cmd := fmt.Sprintf(`{"T":1,"L":%.2f,"R":%.2f}`, left, right)
	return d.SendJSON(cmd)
}

/*
Set the Power going to the wheels.

left and right must be between -255 and 255, if out of bounds they are set to bound.
*/
func (d *UGVDriver) SetPowerToWheels(left, right int16) error {
	if left > 255 {
		left = 255
	}
	if left < -255 {
		left = -255
	}
	if right > 255 {
		right = 255
	}
	if right < -255 {
		right = -255
	}

	cmd := fmt.Sprintf(`{"T":11,"L":%d,"R":%d}`, left, right)
	return d.SendJSON(cmd)
}

func (d *UGVDriver) sendCommand(cmd []byte) error {
	mu.Lock()
	defer mu.Unlock()

	log.Printf("Sending %s.\n", cmd)

	port, err := serial.Open(d.Device, &d.Mode)
	if err != nil {
		log.Println("Error while sending command:", err)
		return err
	}
	defer port.Close()

	n, err := port.Write(append(cmd, '\n'))
	if err != nil {
		log.Println("Error while sending command:", err)
		return err
	}

	log.Printf("Sent %d bytes to serial port.\n", n)

	return nil
}
