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