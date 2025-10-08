package main

import (
	"time"

	"github.com/ets-log795/robot-code/drivers"
	"go.bug.st/serial"
)

func main() {
	driver := &drivers.UGVDriver{
		Device: "/dev/ttyTHS1",
		Mode: serial.Mode{
			BaudRate: 115200, // Example baud rate
			Parity:   serial.NoParity,
			DataBits: 8,
			StopBits: serial.OneStopBit,
		},
	}

	driver.SetSpeed(0.2, 0.2)
	time.Sleep(time.Second)
	driver.SetSpeed(0, 0)

	fdbk, err := driver.GetBaseFeedback()
	if err != nil {
		// Handle error
	}
	println(fdbk)
}
