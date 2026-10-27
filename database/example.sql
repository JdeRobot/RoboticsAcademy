SELECT pg_catalog.setval ('public.exercises_id_seq', (SELECT MAX(id) FROM public.exercises));
SELECT pg_catalog.setval ('public.exercises_tools_id_seq', (SELECT MAX(id) FROM public.exercises_tools));
SELECT pg_catalog.setval ('public.exercises_worlds_id_seq', (SELECT MAX(id) FROM public.exercises_worlds));

INSERT INTO exercises (exercise_id, name, description, tags, entrypoints, status, url)
VALUES (
    'hri_sound',
    'HRI sound',
    'Vacuum Audio In&Out exercise',
    '["ROS2"]',
    '[]',
    'ACTIVE',
    'https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/basic_computer_vision'
);

INSERT INTO exercises_tools (exercise_id, tool_id) VALUES (35, 'console');
INSERT INTO exercises_tools (exercise_id, tool_id) VALUES (35, 'web_gui');
INSERT INTO exercises_tools (exercise_id, tool_id) VALUES (35, 'simulator');
INSERT INTO exercises_worlds (exercise_id, world_id, is_default) VALUES (35, 1, 't')