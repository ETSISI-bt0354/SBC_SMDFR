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
