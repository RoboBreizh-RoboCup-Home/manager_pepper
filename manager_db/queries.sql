SELECT * FROM person WHERE id = :1;

-- Get id from last person added
SELECT id from person order by id DESC limit 1;

-- Get row from person given id
SELECT person.name, person.favorite_drink, person.gender, person.age, color_cloth.label as cloth_color_id, color_skin.label as skin_color_id
FROM person
LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id
LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id
WHERE person.id = 1;

-- Get last person added
SELECT person.id, person.name, person.favorite_drink, person.gender, color_skin.label as skin_color_id, color_cloth.label as cloth_color_id 
FROM person
LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id
LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id
order by person.id DESC limit 1;

SELECT person.name, person.favorite_drink, person.gender, person.age, color_cloth.label as cloth_color_id, color_skin.label as skin_color_id 
FROM person LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id 
LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id 
order by person.id DESC limit 1;

SELECT id FROM color WHERE label = "White";

-- delete all rows from person
DELETE FROM person WHERE id IN (SELECT id FROM person);

-- get last person with name information
SELECT person.name, person.favorite_drink, person.gender, person.age, color_cloth.label as cloth_color_id, color_skin.label as skin_color_id 
FROM person 
LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id 
LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id 
WHERE name IS NOT NULL
order by person.id 
DESC limit 1;

-- get seated person informations
SELECT person.name, person.favorite_drink, person.gender, person.age, color_cloth.label as cloth_color_id, color_skin.label as skin_color_id 
FROM seated_person 
LEFT JOIN person ON seated_person.person_id = person.id
LEFT JOIN color color_cloth ON person.cloth_color_id = color_cloth.id 
LEFT JOIN color color_skin ON person.skin_color_id = color_skin.id 
order by person.id
;

-- get person pose
SELECT person.position_x, person.position_y, person.distance
FROM person
order by person.id;

-- get person pose with associated object
SELECT person.position_x, person.position_y, person.distance
FROM person
LEFT JOIN jt_person_object as jt
LEFT JOIN object
order by person.id;

-- insert object with person id

-- insert person with position