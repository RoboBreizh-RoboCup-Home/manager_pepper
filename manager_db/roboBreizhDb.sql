BEGIN TRANSACTION;
COMMIT;
CREATE TABLE IF NOT EXISTS gpsr_action (
    id INTEGER PRIMARY KEY NOT NULL,
    intent TEXT NOT NULL,
    object_item TEXT,
    person TEXT,
    destination TEXT,
    source TEXT
);

CREATE TABLE IF NOT EXISTS stickler(
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    shoes INTEGER,
    drink INTEGER,
    forbiddenRoom INTEGER,
  	littering INTEGER
);

CREATE TABLE color (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	label TEXT UNIQUE NOT NULL
);

CREATE TABLE IF NOT EXISTS room(
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	label TEXT UNIQUE NOT NULL
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
    angle REAL NOT NULL,
    room_id INTEGER NOT NULL,
    FOREIGN KEY(room_id) REFERENCES room(id)
);

CREATE TABLE IF NOT EXISTS object (
	id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
	label TEXT NOT NULL,
	color_id INTEGER,
    x REAL,
    y REAL,
    z REAL,
    distance REAL,
    room_id INTEGER NOT NULL,
    FOREIGN KEY(room_id) REFERENCES room(id),
    FOREIGN KEY(color_id) REFERENCES color(id)
);

CREATE TABLE IF NOT EXISTS person (
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    name TEXT,
    favorite_drink TEXT,
    gender TEXT,
    age TEXT,
    clothes_style TEXT,
    cloth_color_id INTEGER,
    skin_color_id INTEGER,
    posture TEXT,
    height REAL,
    x REAL,
    y REAL,
    z REAL,
    distance REAL,
    is_drink BOOLEAN NOT NULL,
    is_shoes BOOLEAN NOT NULL,
    FOREIGN KEY(cloth_color_id) REFERENCES color(id),
    FOREIGN KEY(skin_color_id) REFERENCES color(id)
);

CREATE TABLE IF NOT EXISTS jt_person_object(
    person_id integer,
    object_id integer,
    PRIMARY KEY(person_id, object_id)
    FOREIGN KEY(person_id) REFERENCES person(id)
    FOREIGN KEY(object_id) REFERENCES person(id)
);

CREATE TABLE IF NOT EXISTS dialog(
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    run BOOLEAN NOT NULL
);

CREATE TABLE IF NOT EXISTS speech(
    id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    transcript TEXT NOT NULL
);