-- Insert colors
INSERT INTO color (label) 
VALUES ("black"), ("white"), ("brown"), ("yellow"), ("blue"), ("green");

-- Insert object type
INSERT INTO object_type (label) 
VALUES ("furniture"), ("kitcheware"), ("food"), ("drinks");

-- Insert location
INSERT INTO location (name,frame,x,y,z,qw,qx,qy,qz)
VALUES ("instructionPoint","map", 2.092, -0.734, 0.000, 0.000, 0.000, 0.927, 0.376);
INSERT INTO location (name,frame,x,y,z,qw,qx,qy,qz)
VALUES ("arena","map",1.076, 0.770, 0.000,0.000, 0.000, 0.866, 0.499);

-- Insert object
INSERT INTO object (label, type, color) 
VALUES ("can", 3, 1);

-- Insert person
INSERT INTO person (age, height, color_id, outfit)
VALUES (16,1.7,1,"sweater");
