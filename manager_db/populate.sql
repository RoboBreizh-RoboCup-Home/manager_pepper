INSERT INTO person (face_features)
VALUES ("[2,2,3,3]" );

INSERT INTO location (name,frame,x,y,z,qw,qx,qy,qz)
VALUES ("instructionPoint","map", 2.092, -0.734, 0.000, 0.000, 0.000, 0.927, 0.376);
INSERT INTO location (name,frame,x,y,z,qw,qx,qy,qz)
VALUES ("arena","map",1.076, 0.770, 0.000,0.000, 0.000, 0.866, 0.499);


INSERT INTO color (label) 
VALUES ("black"), ("white"), ("brown"), ("yellow"), ("blue"), ("green");

INSERT INTO object_type (label) 
VALUES ("furniture"), ("kitcheware"), ("food"), ("drinks");

INSERT INTO object (label, type, color) 
VALUES ("can", 3, 1);


INSERT INTO person (age, height, color, outfit)
VALUES (16,1.7,1,"sweater");
