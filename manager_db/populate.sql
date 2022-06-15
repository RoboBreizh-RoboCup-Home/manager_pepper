-- Insert colors
INSERT INTO color (label) 
VALUES ("black"), ("white"), ("brown"), ("yellow"), ("blue"), ("green");

-- Insert object type
INSERT INTO object_type (label) 
VALUES ("furniture"), ("kitcheware"), ("food"), ("drinks");

-- Insert location
INSERT INTO location (name,frame,x,y,z,qw,qx,qy,qz)
VALUES ("instructionPoint","map", 2.092, -0.734, 0.000, 0.000, 0.000, 0.927, 0.376),
("arena","map",1.076, 0.770, 0.000,0.000, 0.000, 0.866, 0.499);

-- Insert object
INSERT INTO object (label, object_type_id, color_id) 
VALUES ("can", 3, 1);

-- Insert person
INSERT INTO person (gender, age,cloth_color_id, skin_color_id)
VALUES ("male",25,1,1);