package main

import (
	"time"

	"github.com/ets-log795/robot-code/drivers"
	"go.bug.st/serial"
)

func messageReceived(msg string) {
	println("Received message:", msg)
}

func main() {
	driver := &drivers.UGVDriver{
		Device: "/dev/ttyTHS1",
		Mode: serial.Mode{
			BaudRate: 115200, // Example baud rate
			Parity:   serial.NoParity,
			DataBits: 8,
			StopBits: serial.OneStopBit,
		},
		Callback: messageReceived,
	}

	err := driver.Init()
	defer driver.Close()
	if err != nil {
		println("Error initializing driver:", err)
	}

	driver.SetSpeed(0.2, 0.2)
	time.Sleep(100 * time.Millisecond)
	driver.SetSpeed(0, 0)

	fdbk, err := driver.GetBaseFeedback()
	if err != nil {
		println("Error getting base feedback:", err)
	}
	println(fdbk)

	for range 10 {
		_, err := driver.GetIMUData()
		if err != nil {
			println("Error getting IMU data:", err)
		}
		// println(data)
		time.Sleep(time.Second)
	}
}
