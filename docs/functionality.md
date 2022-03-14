

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

### 2. User code storage in S3.
When the user reopens an exercise, it keeps the code of the last session, for this this code has been stored in S3, some Amazon servers.
The corresponding html document of the exercise executes the function:

![image](https://user-images.githubusercontent.com/60990208/155326543-ceff9f9a-c6f3-4532-967e-c7d0a3d27de0.png)

This function stores the exercise code in the "downloaded_code" variable, obtaining it in turn from the downloadUserCode() function defined in utils.js, located at:
 "RoboticsAcademy\static\assets\common\js".

 ![image](https://user-images.githubusercontent.com/60990208/155327026-329263fe-0b9b-49a6-ad18-9a9ee087ce6c.png)

This function targets the url "/academy/exercise/reload_code/ + "exercise" + /". Checking this url in the urls directory, we see the following:

![image](https://user-images.githubusercontent.com/60990208/155327809-f0ec1c3f-ecc0-4416-a61f-880db7b8a9f2.png)

which leads us to the following view:

![image](https://user-images.githubusercontent.com/60990208/155328006-4865c408-3614-4264-a057-313cf73851fe.png)

Here you can see that an http is returned with the user's aws_pull_file information.
This information is defined in the database model in models.py:

![image](https://user-images.githubusercontent.com/60990208/155328725-3c768208-e78c-426f-ab72-cd5a00aa7415.png)

The boto3 library is used to obtain the required information.


### 3. Funcionamiento del remote-backend.
Normally RADI is on the same machine as the browser, and therefore communication via websockets can be direct. But in the case of the remote-backend, things get complicated, this is because now the RADI finds on another machine, which, to avoid hacking, must have an unknown IP. To achieve this, we make the webserver function as an information mediator.

Locating ourselves further into the code, we can find in "base.html", located in the "templates/academy" folder, the radioButton in charge of managing whether the remote-backend is active or not.

![image](https://user-images.githubusercontent.com/60990208/156360806-3e31beb0-bf01-4188-a6c6-bfcb2e7a9d97.png)

Note that this button is checked if the user.online_docker variable is "true".

If we follow the code executed when we press the remote_backend option:

![image](https://user-images.githubusercontent.com/60990208/156361921-8edc6514-0058-4dca-b580-39e2b3978ce8.png)
![image](https://user-images.githubusercontent.com/60990208/156362060-7e967fb3-452c-438a-abe1-e664f39f52dd.png)
![image](https://user-images.githubusercontent.com/60990208/156364983-7f29c1a1-5934-4fdb-b390-54cce9273461.png)

The code inside "set_docker_info" does several things. The first thing is to free the machine from the farm (if one is being used) in the line "user.free_online_docker":

![image](https://user-images.githubusercontent.com/60990208/156365895-cc5e209c-afdc-4153-90bb-b5d2dcfa7348.png)

The second thing is to put the user.online_docker variable, mentioned above; to "true" if the remote-backend has been selected and to "false" otherwise.

![image](https://user-images.githubusercontent.com/60990208/156366367-7156724d-4da0-48e8-9135-7d8771364d5d.png)

Finally, the newly selected docker IP is saved (127.0.0.1 if it is local-backend and empty otherwise), and the user with this updated configuration.

After having followed these steps, we observe that the key is in the user.online_docker variable of the user. Addressing this we observe the following:

![image](https://user-images.githubusercontent.com/60990208/156367153-9be011bb-8142-4b36-a966-3a29ca6c35e1.png)

A help text indicates that if it is "true" when the user starts a simulation it will be connected to a machine in the farm.

Thanks to this we go to load an exercise in "load_exercise" in "views.py", where we observe two differences:

The first of all is the context:

![image](https://user-images.githubusercontent.com/60990208/156376275-d83bf5cd-a902-4546-9ea9-87b09273daf4.png)

and the second is that in case of being remote-backend, a machine from the farm is first assigned:

![image](https://user-images.githubusercontent.com/60990208/156376741-af7e04cf-31c2-44b2-943a-aca329accae1.png)
![image](https://user-images.githubusercontent.com/60990208/156376930-e701ea21-e322-4f29-be4f-505fe6ce27ea.png)
![image](https://user-images.githubusercontent.com/60990208/156377068-8faf4d37-ae46-4bdd-a798-c9c95bba64f1.png)

Once the machine is assigned, we have to see what the webserver does when the simulation starts. To do this, we go to the "connect" button of an exercise and see the following:

![image](https://user-images.githubusercontent.com/60990208/156770968-7041e9b9-27f3-4e2e-82a7-e27e75eeff83.png)

The highlight of these lines of code is the "startSlim()" function, located in the "RoboticsAcademy\static\exercises\static\assets\common\js\launcher_no_gazebo.js" directory.
The first part of the function would be the following:

![image](https://user-images.githubusercontent.com/60990208/156772204-096f5705-7465-44eb-bccd-421b442a456d.png)
Here we already see that in case of having the websocket address empty (in the case of remote-backend), it locates the websocket address differently.
Finally, if we continue advancing in the code, we see the declaration of two other websockets using the addresses already created:

![image](https://user-images.githubusercontent.com/60990208/156773602-244b001e-34f9-404d-92ac-1e99cc78fab5.png)

![image](https://user-images.githubusercontent.com/60990208/156773683-b954b365-7cb4-4dda-b255-939a5ee631d4.png)
![image](https://user-images.githubusercontent.com/60990208/156773800-e142290c-ff67-4aaf-ba40-2d00951f41c4.png)

These last two functions are declared in the directories corresponding to each exercise:

![image](https://user-images.githubusercontent.com/60990208/156774510-2f91f016-2c8d-4cdf-be04-b1292ae81f13.png)
