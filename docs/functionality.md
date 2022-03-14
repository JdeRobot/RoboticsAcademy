

### [Back to main README.][]

[Back to main README.]: ../README.md


# Unibotics-Webserver


## Functionality blocks
The implementation of the most important functionalities of the project is explained below.

### 1. Webserver relationship and exercises.
When we carry out deployment 1 we must clone the exercises repository within the unibotics-exercises directory, located in "Unibotics-webserver\jderobot_academy\academy\static\unibotics-exercises", it is here where the information regarding the exercises is found, which more Later use our website when we work on the exercises.

The first thing the server does when we click on an exercise is to go to the url corresponding to the exercise id.

![image](https://user-images.githubusercontent.com/60990208/155314503-d8ff8b75-ce77-4c05-8d5e-714f4d3e8b08.png)

When it performs this action, it consults the urls in the file "urls.py" (RoboticsAcademy/academy/urls.py).

![image](https://user-images.githubusercontent.com/60990208/155315135-631d7c58-e93e-4ad7-94b9-006b3ca9089d.png)

This url directs us to the views.load_exercise view:

![image](https://user-images.githubusercontent.com/60990208/155315501-bb608757-1c02-4dd2-a6dd-f7dd8d8a6177.png)

Here we observe how it obtains the html corresponding to the exercise:

![image](https://user-images.githubusercontent.com/60990208/155315960-a754ad60-9b23-4936-93d7-3ee5ab1bb3a2.png)

settings.EXERCISE_TEMPLATES is defined in the settings.py directory:

![image](https://user-images.githubusercontent.com/60990208/155316426-10821fc5-7842-430e-b732-c00ac597df35.png)
