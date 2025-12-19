
-- ===============================
-- CLEAR OLD DATA
-- ===============================
DELETE FROM external_telemetry;
DELETE FROM device_telemetry;
DELETE FROM devices;
DELETE FROM locations;

-- ===============================
-- PARAMETERS (El Pardo bounding box)
-- ===============================
SET @lat_min = 40.4800;
SET @lat_max = 40.5600;
SET @lon_min = -3.7800;
SET @lon_max = -3.7200;

SET @location_count = 30;

-- ===============================
-- GENERATE LOCATIONS IN EL PARDO
-- ===============================

DROP TEMPORARY TABLE IF EXISTS temp_seq;
CREATE TEMPORARY TABLE temp_seq (n INT);

INSERT INTO temp_seq VALUES (0),(1),(2),(3),(4),(5),(6),(7),(8),(9);

-- temp_seq*temp_seq = 100 rows → enough to generate locations
INSERT INTO locations (latitude, longitude)
SELECT
    @lat_min + RAND() * (@lat_max - @lat_min),
    @lon_min + RAND() * (@lon_max - @lon_min)
FROM information_schema.tables
LIMIT 100; -- @location_count


-- ===============================
-- INSERT DEVICES: 1 per location
-- ===============================

INSERT INTO devices (mac_address, location_id)
SELECT
    UNHEX(
        LPAD(HEX(FLOOR(RAND()*281474976710655)), 12, '0')
    ) AS mac,
    id
FROM locations;

-- ===============================
-- GENERATE DEVICE TELEMETRY
-- 1 week of data: 1 reading every 15 minutes = 672 readings per location
-- ===============================

DROP TEMPORARY TABLE IF EXISTS temp_times;
CREATE TEMPORARY TABLE temp_times (offset_seconds INT);

-- Generate time offsets for 1 week (604,800 seconds)
-- One reading every 15 minutes (900 seconds) = 672 readings
INSERT INTO temp_times
SELECT (@rownum := @rownum + 1) * 900 AS offset_seconds
FROM (SELECT 0 UNION ALL SELECT 1 UNION ALL SELECT 2 UNION ALL SELECT 3 UNION ALL SELECT 4 UNION ALL SELECT 5 UNION ALL SELECT 6 UNION ALL SELECT 7 UNION ALL SELECT 8 UNION ALL SELECT 9) t1,
     (SELECT 0 UNION ALL SELECT 1 UNION ALL SELECT 2 UNION ALL SELECT 3 UNION ALL SELECT 4 UNION ALL SELECT 5 UNION ALL SELECT 6 UNION ALL SELECT 7 UNION ALL SELECT 8 UNION ALL SELECT 9) t2,
     (SELECT 0 UNION ALL SELECT 1 UNION ALL SELECT 2 UNION ALL SELECT 3 UNION ALL SELECT 4 UNION ALL SELECT 5 UNION ALL SELECT 6 UNION ALL SELECT 7 UNION ALL SELECT 8 UNION ALL SELECT 9) t3,
     (SELECT @rownum := -1) r
LIMIT 672;

INSERT INTO device_telemetry (location_id, timestamp, humidity, temperature, co2)
SELECT
    l.id,
    UNIX_TIMESTAMP() - tt.offset_seconds,
    40 + RAND()*30,     -- humidity 40–70%
    10 + RAND()*15,     -- temperature 10–25 °C
    CASE
        WHEN top3.id IS NOT NULL
        THEN 5000 + RAND()*1000  -- CO₂ > 5000 ppm para 3 locations
        ELSE 100 + RAND()*900    -- CO₂ normal para el resto
    END
FROM locations l
CROSS JOIN temp_times tt
LEFT JOIN (
    SELECT id
    FROM locations
    ORDER BY id
    LIMIT 3
) AS top3 ON top3.id = l.id;


-- ===============================
-- GENERATE EXTERNAL TELEMETRY
-- (wind, precipitation, etc) - same timestamps as device telemetry
-- ===============================

INSERT INTO external_telemetry (location_id, timestamp, wind_we, wind_ns, wind_speed, precip, ptype)
SELECT
    l.id,
    UNIX_TIMESTAMP() - tt.offset_seconds,
    (RAND()*2 - 1),       -- west/east wind component
    (RAND()*2 - 1),       -- north/south wind component
    RAND()*5,             -- wind speed 0–5 m/s
    RAND()*4/1000,        -- precip 0–4 mm
    FLOOR(RAND()*3)
FROM locations l
CROSS JOIN temp_times tt;
