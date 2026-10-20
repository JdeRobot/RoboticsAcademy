SELECT pg_catalog.setval ('public.exercises_id_seq', (SELECT MAX(id) FROM public.exercises));

SELECT pg_catalog.setval ('public.exercises_tools_id_seq', (SELECT MAX(id) FROM public.exercises_tools));

INSERT INTO exercises (exercise_id, name, description, tags, entrypoints, status, url)
VALUES (
    'audio_out',
    'Audio Out',
    'Basic Audio Out exercise',
    '["ROS2"]',
    '[]',
    'ACTIVE',
    'https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/basic_computer_vision'
);

INSERT INTO exercises_tools (exercise_id, tool_id) VALUES (36, 'console');
INSERT INTO exercises_tools (exercise_id, tool_id) VALUES (36, 'web_gui');