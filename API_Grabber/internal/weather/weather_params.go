package weather

import (
	"bytes"
	"encoding/json"
	"fmt"
	"net/http"

	common "sbcWeatherGrabber/internal/types"
)

type WeatherResponse struct {
	Ts    []int64 `json:"ts"`
	Units struct {
		WindUSurface        string `json:"wind_u-surface"`
		WindVSurface        string `json:"wind_v-surface"`
		Past3HprecipSurface string `json:"past3hprecip-surface"`
		PtypeSurface        string `json:"ptype-surface"`
	} `json:"units"`
	WindUSurface        []float64 `json:"wind_u-surface"`
	WindVSurface        []float64 `json:"wind_v-surface"`
	Past3HprecipSurface []float64 `json:"past3hprecip-surface"`
	PtypeSurface        []int     `json:"ptype-surface"`
}

func GetWeatherParams(data common.WindyRequest) (*WeatherResponse, error) {
	jsonData, err := json.Marshal(data)

	if err != nil {
		return nil, fmt.Errorf("Failed to marshal JSON: %w", err)
	}

	req, err := http.NewRequest("POST", common.WindyApiURL, bytes.NewBuffer(jsonData))

	if err != nil {
		return nil, fmt.Errorf("Failed to create request: %w", err)
	}

	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Accept", "application/json")

	resp, err := http.DefaultClient.Do(req)

	if err != nil {
		return nil, fmt.Errorf("request failed: %w", err)
	}

	defer resp.Body.Close()

	var WeatherResp WeatherResponse

	if err := json.NewDecoder(resp.Body).Decode(&WeatherResp); err != nil {
		return nil, fmt.Errorf("Failed to decode response: %w", err)
	}

	return &WeatherResp, nil
}
