BEGIN TRANSACTION;
COMMIT;

CREATE TABLE color (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	label TEXT NOT NULL
);

CREATE TABLE object_type (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	label TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS location (
    name TEXT PRIMARY KEY UNIQUE NOT NULL,
    frame TEXT NOT NULL,
    x REAL NOT NULL,
    y REAL NOT NULL,
    z REAL NOT NULL, 
    qw REAL NOT NULL,
    qx REAL NOT NULL, 
    qy REAL NOT NULL, 
    qz REAL NOT NULL,
    angle REAL
);

CREATE TABLE IF NOT EXISTS object (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	label TEXT NOT NULL,
	score REAL,
	object_type_id INTEGER NOT NULL,
	color_id INTEGER,
	position_name INTEGER,
    FOREIGN KEY(object_type_id) REFERENCES object_type(id),
    FOREIGN KEY(color_id) REFERENCES colors(id),
    FOREIGN KEY(position_name) REFERENCES location(name)
);

CREATE TABLE IF NOT EXISTS person (
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    name TEXT,
    favorite_drink TEXT,
    age INTEGER,
    height REAL,
    color_id INTEGER,
    outfit TEXT,
    face_features BLOB,
    FOREIGN KEY(color_id) REFERENCES colors(id)
);