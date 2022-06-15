SELECT * FROM person WHERE id = :1;

SELECT object.id, object.label, colors.label 
FROM object 
INNER JOIN colors 
ON object.color=colors.id;

SELECT id from person order by id DESC limit 1;
