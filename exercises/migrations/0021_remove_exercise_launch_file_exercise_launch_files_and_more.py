# Generated by Django 4.2.1 on 2023-10-20 11:31

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('exercises', '0020_exercise_launch_file_exercise_model_folders_and_more'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='exercise',
            name='launch_file',
        ),
        migrations.AddField(
            model_name='exercise',
            name='launch_files',
            field=models.TextField(default='{}'),
        ),
        migrations.AlterField(
            model_name='exercise',
            name='model_folders',
            field=models.CharField(default='$CUSTOM_ROBOTS_FOLDER/', max_length=100),
        ),
        migrations.AlterField(
            model_name='exercise',
            name='resource_folders',
            field=models.CharField(default='$EXERCISE_FOLDER/', max_length=100),
        ),
    ]
