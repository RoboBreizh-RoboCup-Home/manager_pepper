PRAGMA foreign_keys=OFF;
BEGIN TRANSACTION;
COMMIT;

CREATE TABLE IF NOT EXISTS person (
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    name TEXT,
    age INTEGER,
    height REAL,
    color colors,
    outfit TEXT,
    position location,
    face_features BLOB
);

CREATE TABLE IF NOT EXISTS object (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	label TEXT NOT NULL,
	score REAL,
	type object_types NOT NULL,
	color colors,
	position location
);

CREATE TABLE IF NOT EXISTS location (
    name TEXT PRIMARY KEY UNIQUE NOT NULL,
    frame TEXT NOT NULL,
    x REAL NOT NULL,
    y REAL NOT NULL,
    z REAL NOT NULL, 
    qx REAL NOT NULL, 
    qy REAL NOT NULL, 
    qz REAL NOT NULL, 
    qw REAL NOT NULL,
);

CREATE TABLE colors (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	label TEXT NOT NULL
);

CREATE TABLE object_type (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	label TEXT NOT NULL
);

INSERT INTO colors (label) VALUES ("black"), ("white"), ("brown"), ("yellow"), ("blue"), ("green");
INSERT INTO object_types (label) VALUES ("furniture"), ("kitcheware"), ("food"), ("drinks");
INSERT INTO object (label, type, color) VALUES ("can", 3, 1);
SELECT object.id, object.label, colors.label FROM object INNER JOIN colors ON object.color=colors.id;