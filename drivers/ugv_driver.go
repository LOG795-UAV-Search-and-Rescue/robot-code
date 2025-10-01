package drivers

import (
	"encoding/json"
	"log"
	"sync"

	"go.bug.st/serial"
)

var mu sync.Mutex

type UGVDriver struct {
	Device string
	Mode   serial.Mode
}

type CMD_SPD_CTRL struct {
	T int     `json:"T"`
	L float32 `json:"L"`
	R float32 `json:"R"`
}

func (d *UGVDriver) SetSpeed(left, right float32) error {
	cmd := CMD_SPD_CTRL{
		T: 1,
		L: left,
		R: right,
	}

	jsonData, err := json.Marshal(cmd)
	if err != nil {
		log.Println("Error marshalling to JSON: ", err)
		return err
	}

	return d.sendCommand(jsonData)
}

func (d *UGVDriver) SendJSON(cmd string) {
	log.Printf("Sent through \"%s\".\n", cmd)
	d.sendCommand([]byte(cmd))
}

func (d *UGVDriver) sendCommand(cmd []byte) error {
	mu.Lock()
	defer mu.Unlock()

	log.Printf("Sending \"%s\".\n", cmd)

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
