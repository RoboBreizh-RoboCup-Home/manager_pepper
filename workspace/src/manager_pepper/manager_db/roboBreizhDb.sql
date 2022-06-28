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
    gender TEXT,
    age TEXT,
    cloth_color_id INTEGER,
    skin_color_id INTEGER,
    face_features BLOB,
    FOREIGN KEY(cloth_color_id) REFERENCES colors(id),
    FOREIGN KEY(skin_color_id) REFERENCES colors(id)
);

CREATE TABLE IF NOT EXISTS seated_person (
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    person_id INTEGER,
    FOREIGN KEY(person_id) REFERENCES person(id)
);

CREATE TABLE IF NOT EXISTS gpsr_action (
    id INTEGER PRIMARY KEY NOT NULL,
    intent TEXT NOT NULL,
    object_item TEXT,
    person TEXT,
    destination TEXT,
    who TEXT,
    what TEXT
);