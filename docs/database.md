
### [Back to main README.][]

[Back to main README.]: ../README.md


# Database

Robotics Academy stores its data in a PostgreSQL database (`academy_db`). The connection is configured in the `DATABASES` setting of [`academy/settings.py`](../academy/settings.py), and the tables of the application are defined by the Django models in [`academy/models.py`](../academy/models.py).

The database is created by the `jderobot/robotics-database` container, which loads these files on its first start (see [Instructions for developers](./InstructionsForDevelopers.md) for the full command):

- `RoboticsInfrastructure/database/worlds.sql`: the `scenes`, `worlds`, `robots`, `tools` and `worlds_robots` tables.
- `database/exercises/db.sql`: the `exercises`, `exercises_worlds` and `exercises_tools` tables.
- `database/django_auth.sql`: the Django authentication, admin, session and migration tables.

To add or edit an exercise use the Django admin site, as explained in [Add the exercise to the database](./InstructionsForDevelopers.md#add-the-exercise-to-the-database).

### Tables

The database has 18 tables: 8 used by Robotics Academy and 10 that belong to Django.

#### Robotics Academy tables

  - exercises: the exercises shown on the home page. Includes 8 columns:
    - id
    - exercise_id: unique identifier, the same as the exercise folder name
    - name
    - description
    - tags
    - entrypoints
    - status: `ACTIVE`, `INACTIVE` or `PROTOTYPE`
    - url

  - worlds: the worlds an exercise can run in. Includes 3 columns:
    - id
    - name
    - scene_id: the scene of the world

  - scenes: the simulation scenes. Includes 7 columns:
    - id
    - name
    - launch_file_path
    - tools_config
    - ros_version: `ROS` or `ROS2`
    - type: `none`, `gazebo`, `gz` or `physical`
    - model_path

  - robots: the robots that can be placed in a world. Includes 6 columns:
    - id
    - name
    - launch_file_path
    - entity
    - extra_config
    - model_path

  - tools: the tools (for example `console`, `simulator` or `web_gui`) an exercise can use. Includes 2 columns:
    - name
    - base_config

  - exercises_worlds: relates exercises and worlds. Includes 4 columns:
    - id
    - exercise_id
    - world_id
    - is_default: whether the world is the default one for the exercise

  - exercises_tools: relates exercises and tools. Includes 3 columns:
    - id
    - exercise_id
    - tool_id

  - worlds_robots: relates worlds and robots. Includes 4 columns:
    - id
    - world_id
    - robot_id
    - poses: starting poses of the robot in the world

#### Django tables

Django's own tables (loaded from `database/django_auth.sql`), used for the admin login, sessions and migrations:

  - auth_group
  - auth_group_permissions
  - auth_permission
  - auth_user
  - auth_user_groups
  - auth_user_user_permissions
  - django_admin_log
  - django_content_type
  - django_migrations
  - django_session
