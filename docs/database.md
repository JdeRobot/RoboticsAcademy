
### [Back to main README.][]

[Back to main README.]: ../README.md


# Data base

The data base of this proyect is localizated en main directory, it's name is db.sqlite3. In this file, we save the structure of all the information that the website is compiling. Db is divided in  12 tables that includes all the diferent parameters. Most of this tables are created and modified by Django, we are just gonna use the table "exercises_exercise".

### Tables

db contains 12 tables that save the information of the website:

  - auth_group: group of authorised groups. Includes 2 columns:
    - id
    - name

  - auth_group_permissions: contains the authorised groups' permissions. Includes 3 columns:
    - id
    - group_id
    - permission_id

  - auth_permission: contains the permissions' list . Includes 4 columns:
    - id
    - content_type_id
    - codename
    - name

  - auth_user: contains a web users' list. Includes 11 columns:
    - id
    - password
    - last_login
    - is_superuser
    - username
    - last_name
    - email
    - is_staff
    - is_active
    - date_joined
    - first_name

  - auth_user_groups: contains the user's groups. Includes 3 columns:
    - id
    - user_id
    - group_id

  - auth_user_user_groups: contains the user's permissions. Includes 3 columns:
    - id
    - user_id
    - permission_id

  - django_admin_log: contains the Django's dates. Includes 8 columns:
    - id
    - action_time
    - object_id
    - object_repr
    - change_message
    - content_type_id
    - user_id
    - action_flag

  - django_content_type: contains the diferent type of information that Django server saves. Includes 3 columns:
    - id
    - app_label
    - model

  - django_migrations: contains the Django's migrations. Includes 4 columns:
    - id
    - app
    - name
    - applied

  - django_session: contains the Django's sessions. Includes 3 columns:
    - session_key
    - session_data
    - expire_date

  - exercises_exercise: contains the web's exercises. Includes 5 columns:
    - id
    - exercise_id
    - name
    - description
    - assets

  - sqlite_sequence: contains the sqlite's sequences. Includes 2 columns:
    - name
    - seq
