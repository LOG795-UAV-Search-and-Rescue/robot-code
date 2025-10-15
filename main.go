package main

import (
	"log"
	"math"
	"time"

	"github.com/ets-log795/robot-code/drivers"
	"go.bug.st/serial"
)

func degToRad(deg float32) float32 {
	return deg * (math.Pi / 180)
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
	}

	err := driver.Init()
	defer driver.Close()
	if err != nil {
		log.Println("Error initializing driver:", err.Error())
	}

	time.Sleep(10 * time.Second)
	err = driver.SendGimbalBasicControls(90, 0, 0, 0)
	if err != nil {
		log.Println("Error sending gimbal controls:", err.Error())
	}

	time.Sleep(10 * time.Second)

	const timeToTurn = 5 // seconds to turn 90 degrees

	log.Println("Turning...")
	err = driver.SendROSControls(0, degToRad(90))
	if err != nil {
		log.Println("Error move controls:", err.Error())
	}

	time.Sleep(time.Second)
	log.Println("Stopping...")
	err = driver.SendROSControls(0, 0)
	if err != nil {
		log.Println("Error move controls:", err.Error())
	}

	time.Sleep(timeToTurn * time.Second)

	err = driver.SendGimbalBasicControls(0, 0, 0, 0)
	if err != nil {
		log.Println("Error sending gimbal controls:", err.Error())
	}

	time.Sleep(10 * time.Second)

	// driver.SetSpeed(0.2, 0.2)

	// time.Sleep(100 * time.Millisecond)

	// for range 10 {
	// 	data, err := driver.GetBaseFeedback()
	// 	if err != nil {
	// 		println("Error getting base feedback:", err.Error())
	// 	}
	// 	println(data)
	// 	time.Sleep(100 * time.Millisecond)
	// }

	// driver.SetSpeed(0, 0)

	// for range 10 {
	// 	data, err := driver.GetBaseFeedback()
	// 	if err != nil {
	// 		println("Error getting base feedback:", err.Error())
	// 	}
	// 	println(data)
	// 	time.Sleep(time.Second)
	// }
}
