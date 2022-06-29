BEGIN TRANSACTION;
COMMIT;

CREATE TABLE color (
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
	color_id INTEGER,
    position_x REAL,
    position_y REAL,
    position_z REAL,
    distance REAL,
    FOREIGN KEY(color_id) REFERENCES colors(id),
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
    position_x REAL,
    position_y REAL,
    position_z REAL,
    distance REAL,
    FOREIGN KEY(cloth_color_id) REFERENCES colors(id),
    FOREIGN KEY(skin_color_id) REFERENCES colors(id)
);

CREATE TABLE IF NOT EXISTS seated_person (
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    person_id INTEGER,
    FOREIGN KEY(person_id) REFERENCES person(id)
);

CREATE TABLE IF NOT EXISTS jt_person_object(
    person_id integer,
    object_id integer,
    PRIMARY KEY(person_id, object_id)
    FOREIGN KEY(person_id) REFERENCES person(id)
    FOREIGN KEY(object_id) REFERENCES person(id)
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