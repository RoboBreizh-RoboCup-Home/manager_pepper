INSERT INTO person (face_features)
VALUES ("[2,2,3,3]" );

INSERT INTO location (name,frame,x,y,z,qw,qx,qy,qz,theta)
VALUES ("instructionPoint","map", 2.092, -0.734, 0.000, 0.000, 0.000, 0.927, 0.376, 2.370);
VALUES ("arena","map",1.076, 0.770, 0.000,0.000, 0.000, 0.866, 0.499,2.096 );


INSERT INTO colors (label) 
VALUES ("black"), ("white"), ("brown"), ("yellow"), ("blue"), ("green");

INSERT INTO object_types (label) 
VALUES ("furniture"), ("kitcheware"), ("food"), ("drinks");

INSERT INTO object (label, type, color) 
VALUES ("can", 3, 1);