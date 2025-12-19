package main

import (
	"encoding/binary"
	"fmt"
	"log"
	"math"
	"net"
	"os"
	"time"

	"database/sql"
	_ "github.com/go-sql-driver/mysql"
)

type Telemetry struct {
	TimeStamp   time.Time
	Mac         [6]byte
	Humidity    float32
	Temperature float32
	Co2         uint16
}

func main() {
	port, exist := os.LookupEnv("PORT")
	if !exist {
		log.Fatal("PORT env variable not set")
	}

	addr, err := net.ResolveUDPAddr("udp", ":"+port)
	if err != nil {
		log.Fatal("Error resolving UDP address: ", err)
	}

	conn, err := net.ListenUDP("udp", addr)
	if err != nil {
		log.Fatal("Error listening on UDP: ", err)
	}
	defer conn.Close()

	log.Println("UDP server listening on ", port)

	telemetry_channel := make(chan Telemetry, 100)
	go storeTelemetry(telemetry_channel)

	for {
		buf := make([]byte, 1024)
		n, _, err := conn.ReadFrom(buf)
		if err != nil {
			log.Println("Error reading UDP message: ", err)
			continue
		}

		go handleMessage(buf[:n], telemetry_channel)
	}
}

func handleMessage(buf []byte, c chan Telemetry) {
	var msg Telemetry

	log.Printf("Receive %d bytes", len(buf))
	if len(buf) < 16 {
		fmt.Println("Message size to small")
		return
	}

	msg.TimeStamp = time.Now()
	copy(msg.Mac[:], buf[0:6])
	humidity_bytes := binary.LittleEndian.Uint32(buf[6:10])
	msg.Humidity = math.Float32frombits(humidity_bytes)
	temperature_bytes := binary.LittleEndian.Uint32(buf[10:14])
	msg.Temperature = math.Float32frombits(temperature_bytes)
	msg.Co2 = binary.LittleEndian.Uint16(buf[14:16])

	c <- msg
}

func storeTelemetry(c chan Telemetry) {
	db, err := sql.Open("mysql", getMySQLDSN())
	if err != nil {
		log.Fatal("Error connecting to the database: ", err)
	}
	defer db.Close()

	db.SetConnMaxLifetime(time.Minute * 3)
	db.SetMaxOpenConns(10)
	db.SetMaxIdleConns(10)

	for msg := range c {
		log.Printf("{Timestamp: %v, Mac: %x, Humidity: %v, Temperature: %v, CO2: %v}\n",
			msg.TimeStamp.Format(time.UnixDate),
			msg.Mac,
			msg.Humidity,
			msg.Temperature,
			msg.Co2)

		var deviceId uint64
		row := db.QueryRow("SELECT location_id FROM devices WHERE mac_address = ?", msg.Mac[:])
		if err := row.Scan(&deviceId); err != nil {
			if err == sql.ErrNoRows {
				log.Printf("Device with %x mac is not recognise\n", msg.Mac)
			}

			log.Println("Unexpected sql error: ", err)
			return
		}

		log.Println("Devices found, id: ", deviceId)

		_, err := db.Exec("INSERT INTO device_telemetry (location_id, timestamp, humidity, temperature, co2) VALUES (?, ?, ?, ?, ?)",
			deviceId,
			msg.TimeStamp.Unix(),
			msg.Humidity,
			msg.Temperature,
			msg.Co2)
		if err != nil {
			log.Println("Error while inserting telemetry: ", err)
			return
		}

		log.Println("Telemetry successfully inserted")
	}
}

func getMySQLDSN() string {
	const (
		DefaultDBUser     = "root"
		DefaultDBPassword = "root"
		DefaultDBHost     = "127.0.0.1"
		DefaultDBPort     = "3306"
		DefaultDBName     = "telemetry"
	)

	user := getEnvDefault("DB_USER", DefaultDBUser)
	pass := getEnvDefault("DB_PASSWORD", DefaultDBPassword)
	host := getEnvDefault("DB_HOST", DefaultDBHost)
	port := getEnvDefault("DB_PORT", DefaultDBPort)
	name := getEnvDefault("DB_NAME", DefaultDBName)

	return fmt.Sprintf("%s:%s@tcp(%s:%s)/%s?parseTime=true", user, pass, host, port, name)
}

func getEnvDefault(key, defaultValue string) string {
	value, exists := os.LookupEnv(key)

	if !exists {
		value = defaultValue
	}

	return value
}
