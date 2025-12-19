package common

const (
	DefaultLat   = 40.4226048
	DefaultLon   = -3.6896768
	DefaultModel = "arome"
	DefaultLevel = "surface"
)

const (
	DefaultDBUser     = "root"
	DefaultDBPassword = "root"
	DefaultDBHost     = "127.0.0.1"
	DefaultDBPort     = "3306"
	DefaultDBName     = "telemetry"
)

const WindyApiURL = "https://api.windy.com/api/point-forecast/v2"

type WindyRequest struct {
	Lat        float64  `json:"lat"`
	Lon        float64  `json:"lon"`
	Model      string   `json:"model"`
	Parameters []string `json:"parameters"`
	Levels     []string `json:"levels"`
	Key        string   `json:"key"`
}
