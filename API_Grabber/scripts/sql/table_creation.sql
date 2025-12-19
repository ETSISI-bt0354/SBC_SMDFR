DROP DATABASE telemetry;

CREATE DATABASE IF NOT EXISTS telemetry;
USE telemetry;

CREATE TABLE IF NOT EXISTS locations(
    id INTEGER PRIMARY KEY AUTO_INCREMENT,
    latitude DECIMAL(9, 7) NOT NULL,
    longitude DECIMAL(11, 8) NOT NULL,
    UNIQUE (latitude, longitude)
);

CREATE TABLE IF NOT EXISTS devices(
    id INTEGER PRIMARY KEY AUTO_INCREMENT,
    mac_address BINARY(6) NOT NULL UNIQUE,
    location_id INTEGER NOT NULL,
    FOREIGN KEY (location_id) REFERENCES locations(id)
);

CREATE TABLE IF NOT EXISTS device_telemetry (
    id INTEGER PRIMARY KEY AUTO_INCREMENT,
    location_id INTEGER NOT NULL,
    timestamp INTEGER NOT NULL,
    humidity REAL NOT NULL,
    temperature REAL NOT NULL,
    co2 REAL NOT NULL,
    FOREIGN KEY (location_id) REFERENCES locations(id)
);

CREATE TABLE IF NOT EXISTS external_telemetry (
    id INTEGER PRIMARY KEY AUTO_INCREMENT,
    location_id INTEGER NOT NULL,
    timestamp INTEGER NOT NULL,
    wind_we REAL NOT NULL,
    wind_ns REAL NOT NULL,
    wind_speed REAL NOT NULL,
    precip REAL NOT NULL,
    ptype INTEGER NOT NULL,
    UNIQUE (location_id, timestamp),
    FOREIGN KEY (location_id) REFERENCES locations(id)
);

CREATE INDEX idx_dt_location_timestamp
    ON device_telemetry(location_id, timestamp);

CREATE INDEX idx_et_location_timestamp
    ON external_telemetry(location_id, timestamp);

