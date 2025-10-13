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

	err := driver.Init()
	defer driver.Close()
	if err != nil {
		println("Error initializing driver:", err.Error())
	}

	// driver.SetSpeed(0.2, 0.2)
	// time.Sleep(100 * time.Millisecond)
	// driver.SetSpeed(0, 0)

	for range 10 {
		data, err := driver.GetBaseFeedback()
		if err != nil {
			println("Error getting base feedback:", err.Error())
		}
		println(data)
		time.Sleep(time.Second)
	}
}
