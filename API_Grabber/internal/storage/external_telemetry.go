package external_telemetry

import (
	"database/sql"
	"fmt"
	"log"
	"math"
	"os"
	"time"

	_ "github.com/go-sql-driver/mysql"

	common "sbcWeatherGrabber/internal/types"
	"sbcWeatherGrabber/internal/weather"
)

type externalTelemetry struct {
	ID        int64
	TimeStamp time.Time
	WindWE    float64
	WindNS    float64
	WindSpeed float64
	Precip    float64
	Ptype     int
}

func getEnvDefault(key, defaultValue string) string {
	value, exists := os.LookupEnv(key)

	if !exists {
		value = defaultValue
	}

	return value
}

func getMySQLDSN() string {
	user := getEnvDefault("DB_USER", common.DefaultDBUser)
	pass := getEnvDefault("DB_PASSWORD", common.DefaultDBPassword)
	host := getEnvDefault("DB_HOST", common.DefaultDBHost)
	port := getEnvDefault("DB_PORT", common.DefaultDBPort)
	name := getEnvDefault("DB_NAME", common.DefaultDBName)

	return fmt.Sprintf("%s:%s@tcp(%s:%s)/%s?parseTime=true", user, pass, host, port, name)
}

func GrabAndStoreTelemetry() {
	db, err := sql.Open("mysql", getMySQLDSN())

	if err != nil {
		log.Fatal("Error connecting to the database: ", err)
	}

	defer db.Close()

	db.SetConnMaxLifetime(time.Minute * 3)
	db.SetMaxOpenConns(10)
	db.SetMaxIdleConns(10)

	rows, err := db.Query("SELECT id, latitude, longitude FROM locations")

	if err != nil {
		log.Fatal("Error when selecting location from the database: ", err)
	}

	defer rows.Close()

	for rows.Next() {
		var id int64
		var lat, lon float64

		if err := rows.Scan(&id, &lat, &lon); err != nil {
			log.Fatal("Error scanning rows: ", err)
		}

		requestData := common.WindyRequest{
			Lat:        lat,
			Lon:        lon,
			Model:      common.DefaultModel,
			Parameters: []string{"wind", "precip", "ptype"},
			Levels:     []string{common.DefaultLevel},
			Key:        os.Getenv("WINDY_API_KEY"),
		}

		response, err := weather.GetWeatherParams(requestData)

		if err != nil {
			log.Fatal("Error obtaining data from API: ", err)
		}

		log.Printf("Received %d forecast data\n", len(response.Ts))

		for i := range response.Ts {
			wind_u := response.WindUSurface[i]
			wind_v := response.WindVSurface[i]

			msg := externalTelemetry{
				ID:        id,
				TimeStamp: time.UnixMilli(response.Ts[i]),
				WindWE:    wind_u,
				WindNS:    wind_v,
				WindSpeed: math.Hypot(wind_u, wind_v),
				Precip:    response.Past3HprecipSurface[i] * 1000.0,
				Ptype:     response.PtypeSurface[i],
			}

			stmt, err := db.Prepare(`
    			INSERT INTO external_telemetry (
        			location_id, timestamp, wind_we, wind_ns, wind_speed, precip, ptype
           		) VALUES (?, ?, ?, ?, ?, ?, ?)
             	ON DUPLICATE KEY UPDATE
             		wind_we = VALUES(wind_we),
              		wind_ns = VALUES(wind_ns),
               		wind_speed = VALUES(wind_speed),
                 	precip = VALUES(precip),
                  	ptype = VALUES(ptype)
                `)

			if err != nil {
				log.Fatal("Error creating prepared statement: ", err)
			}

			defer stmt.Close()

			_, err = stmt.Exec(msg.ID, msg.TimeStamp.Unix(), msg.WindWE, msg.WindNS, msg.WindSpeed, msg.Precip, msg.Ptype)

			if err != nil {
				log.Fatal("Error inserting external telemetry: ", err)
			}

			log.Println("External telemetry succesfully inserted")
		}
	}
}
