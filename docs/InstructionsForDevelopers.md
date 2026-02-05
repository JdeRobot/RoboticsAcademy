### [Back to main README.][]

[Back to main README.]: ../README.md

# Instructions for developers

- [Getting started with Robotics Academy for developers](https://youtu.be/3AM-ztcRsr4)
- [How to setup the developer environment](#How-to-setup-the-developer-environment)
  - [Developer environment set up via script _(recommended set up)_](#automatic-script)
  - [Developer environment set up via docker-compose _(recommended set up for **Windows Users**)_](#docker-compose)
- [How to use nvidia](#How-to-use-nvidia)
- [How to add a new exercise](#How-to-add-a-new-exercise)
- [Steps to change models from CustomRobots in RoboticsAcademy exercises](#Steps-to-change-models-from-CustomRobots-in-RoboticsAcademy-exercises)

<a name="How-to-setup-the-developer-environment"></a>

## How to setup the developer environment

Before starting developing, please ensure that you have understood RoboticsAcademy architecture and where the different resources are placed. There are three different ways of developing in RA:

<a name="automatic-script"></a>

### Using automatic script (recommended)

We provide an sh script that configures and runs automatically a developing environment:

1. Clone RA repo

```
git clone --recurse-submodules https://github.com/JdeRobot/RoboticsAcademy.git -b <src-branch>
cd RoboticsAcademy/
```

You can ignore the -b arg if you want to start working from the main branch.

2. Run the script with your desired config

```
./scripts/develop_academy.sh -r <link to the RAM repo/fork> -b <branch of the RAM repo> -i <humble>
```

If you don't provide any arguments, it will prepare a humble environment with the current stable branch of RAM. You may start working from that and then create the branch you need.
You may access RA frontend at [http://127.0.0.1:7164/academy/](http://127.0.0.1:7164/academy/)

\
If you need more information about the options available for launching the script, you can use:

```
./scripts/develop_academy.sh -h
```

Which will display a help message.

3. Developing procedure

After running the script, the src folder will be created, which contains all the files of the RoboticsApplicationManager. You can create branches and commit normally to the RAM repo from inside that folder. For the rest of the changes, you can also work normally from the RoboticsAcademy folder, the contents of the src folder are automatically ignored.

Whenever you want to finish developing, you just can close the script with Crtl+C. It will take care of cleaning files so you can restart again without any additional config.

**Note: For Apple M1/M2 Chip Users,Docker provides the feature in-built in docker Desktop to use Rosetta**

Go to **Settings** > **General** :
Enable Use Rosetta for x86_64/amd64 emulation on Apple Silicon

- [x] _Use Rosetta for x86_64/amd64 emulation on Apple Silicon_

Please look at the attached image for reference.

<img width="1440" alt="Screenshot 2024-05-01 at 10 35 55 PM" src="https://github.com/JdeRobot/RoboticsAcademy/assets/57873504/c4096ab4-f9c1-4ddf-b612-41e78074fb99">

### Some problems that can arise

It is possible that the first time you follow the instructions, a dependency may not be installed correctly, or it may not be added to the path for some reason.

One of the most frequent problems is that the frontend doesn't launch, you can solve it in two ways, the first one is to launch the frontend separately from another terminal:

```
cd /RoboticsAcademy
```

```
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
nvm install 20
nvm use 20
npm install --global yarn
cd react_frontend/ && yarn install && yarn run dev
```

Another way to solve it is to try to delete the old RADI image and download it again.

<a name="docker-compose"></a>

### Using Docker compose (Recommended for Windows users)

**_NOTE_**: If you are following this tutorial as a Windows users, please follow all these steps but using WSL (Linux kernel for Windows). Please visit the next [link](https://learn.microsoft.com/en-us/windows/wsl/install) in case you don't have WSL installed.

Docker Compose is a tool for defining and running multi-container applications. It is the key to unlocking a streamlined and efficient development and deployment experience. Compose makes easy to manage services, networks, and volumes in a single, comprehensible YAML configuration file. Then, with a single command, you create and start all the services from your configuration file. In this YAML file we provide all the configurations needed for a smooth development experience, mainly ports and volumes. This method works by binding your local folder to the appropiate place inside a RoboticsBackend container, where all the dependencies are installed.

The steps for setting up a development environment using Docker Compose are:

1. Install Docker Compose

```
sudo apt install docker-compose
```

2. Clone RoboticsAcademy repo (or your fork) and create src folder

```
git clone --recurse-submodules https://github.com/JdeRobot/RoboticsAcademy.git -b <src-branch>
cd RoboticsAcademy/
```

3. Clone RAM repo (or your fork) inside RA

```
git clone https://github.com/JdeRobot/RoboticsApplicationManager.git -b <src-branch> src
```

For the moment, the RAM folder MUST be called src, and the previous command takes care of that. You can create branches and commits from that folder without any issues.

4. Creation _commons.zip_

In order for the front-end to build, you need to manually create the commons zip, that will be used to pass those files to the Robotics Backend.

```
# Prepare the commons zip file
cd common
cd console_interfaces
zip -r ../common.zip console_interfaces/
cd ..
cd gui_interfaces
zip -r -u ../common.zip gui_interfaces/
cd ..
cd hal_interfaces
zip -r -u ../common.zip hal_interfaces/
cd ../..
mv common/common.zip react_frontend/src/common.zip
```

5. Build the REACT frontend. You **must** do this each time

```
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
nvm install 20
nvm use 20
npm install --global yarn
cd react_frontend/ && yarn install && yarn run dev
```

Please take into consideration that the `yarn run dev` script will continously watch for changes in the frontend, so you should execute this commands in a separate terminal.

6. Copy the desired compose config into the main RA folder

```
cp compose_cfg/<your desired compose cfg> docker-compose.yaml
```

Feel free to study the configs, and adapt/create new ones suitable for your needs.

**_NOTE_**: As a Windows user, if you are willing to use GPU acceleration, there is a docker-compose file prepared for that BUT ONLY AVAILABLE with Nvidia GPUs (and WSL). Visit the following links [WSL + CUDA](https://learn.microsoft.com/en-us/windows/ai/directml/gpu-cuda-in-wsl), [WSL + Docker Desktop](https://docs.docker.com/desktop/features/wsl) to set-up Nvidia CUDA on WSL.

7. Start Docker Compose

```
docker-compose up
```

Now you can open the RoboticsAcademy folder in your preferred code editor and test the changes inside the docker without having to regenerate a new image. Please keep in mind that this method works using a given RoboticsBackend version as the base. The only difference for developing between RoboticsBackend versions is the ROS version (humble) and the branch of RoboticsInfrastructure. If you need to make changes in RI, we recommend that you follow [this procedure](##edit-code-on-RoboticsBackend-on-the-go).

After testing the changes, you can simply commit them from the RA repo. Please keep in mind that the changes in RAM inside the src folder won't be commited, as they are not part of RoboticsAcademy. To commit those changes, just get inside the src/ folder and work from there (remember, this is the RAM repo with another name).

6. Stop docker compose

```
docker-compose down
```

When you finish developing, you can close the container with Ctrl+C, but after that, you must clean the environment executing the previous command, otherwise, some things may not work in the next execution.

<!-- **Note: How to update Robotics Academy local deployment with Node 20 and sass**

Robotics Academy has been updated to use Node 20 and sass. If you have a Robotics Academy local deployment and you don't want to make a new one, you can follow the next instructions to update your local deployment in order to use both dependencies:

1. Go into RoboticsAcademy folder

```
cd RoboticsAcademy/
```

2. Pull the new changes from Robotics Academy humble-devel branch into your local branch
3. Install and use Node 20

```
nvm install 20
nvm use 20
```

4. Reinstall yarn and rebuild the REACT frontend

```
cd react_frontend/
yarn install
yarn run dev
```

Now, you can continue using your local deployment with Node 20 and sass.

**Note:** If you have problems during this process, use the following command before installing Node 20:

```
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
```

**Note:** This steps are not necessary if you deploy Robotics Academy in developer mode using an automatic script. When the script is executed, it internally runs the commands. -->

### Using Docker run: **Only** use this method if the ones before have failed.

If you are launching Robotics Academy this way you need to manually create the commons zip, that will be used to pass those files to the Robotics Backend.

```
# Prepare the commons zip file
cd common
cd console_interfaces
zip -r ../common.zip console_interfaces/
cd ..
cd gui_interfaces
zip -r -u ../common.zip gui_interfaces/
cd ..
cd hal_interfaces
zip -r -u ../common.zip hal_interfaces/
cd ../..
mv common/common.zip react_frontend/src/common.zip
```

You **must** compile the frontend each time using:

```
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
nvm install 20
nvm use 20
npm install --global yarn
cd react_frontend/ && yarn install && yarn run dev
```

You also have to download the Robotics Applictaion Manager using:

```
git clone https://github.com/JdeRobot/RoboticsApplicationManager.git -b humble-devel src
```

You have 2 ways of launching Robotics Academy with docker run:

- Creating a new RADI. To see how to do it and why to use it, read [how to generate a RADI][].
- Using the docker image: `robotics-academy:latest`

Then to launch Robotics Academy first you have to launch the database docker container. For example if you want to launch it from where you cloned Robotics Academy you can use the next command:

**_NOTE_**: If you are in another folder you may need to change the first part of the paths of the volume bindings (**-v**) to the correct path.

```bash
docker run --hostname my-postgres --name academy_db -d\
    -e POSTGRES_DB=academy_db \
    -e POSTGRES_USER=user-dev \
    -e POSTGRES_PASSWORD=robotics-academy-dev \
    -e POSTGRES_PORT=5432 \
    -d -p 5432:5432 \
    -v ./RoboticsInfrastructure/database/universes.sql:/docker-entrypoint-initdb.d/1.sql \
    -v ./database/exercises/db.sql:/docker-entrypoint-initdb.d/2.sql \
    -v ./database/django_auth.sql:/docker-entrypoint-initdb.d/3.sql \
    jderobot/robotics-database:latest
```

Now you can launch Robotics Academy using the followings commands:

**_NOTE_**: If you are in another folder you may need to change the first part of the paths of the volume bindings (**-v**) to the correct path.

- Automatic GPU selection

```bash
docker run --rm -it $(nvidia-smi >/dev/null 2>&1 && echo "--gpus all" || echo "") --device /dev/dri -p 6080-6090:6080-6090 -p 7163:7163 -p 7164:7164 --link academy_db -v ./:/RoboticsAcademy -v ./src:/RoboticsApplicationManager jderobot/robotics-academy:latest
```

- Automatic GPU selection (Without Nvidia)

```bash
docker run --rm -it --device /dev/dri -p 6080-6090:6080-6090 -p 7163:7163 -p 7164:7164 --link academy_db -v ./:/RoboticsAcademy -v ./src:/RoboticsApplicationManager jderobot/robotics-academy:latest
```

- Only CPU

```bash
docker run --rm -it -p 6080-6090:6080-6090 -p 7163:7163 -p 7164:7164 --link academy_db -v ./:/RoboticsAcademy -v ./src:/RoboticsApplicationManager jderobot/robotics-academy:latest
```

[how to generate a RADI]: ./generate_a_radi.md

<a name="How-to-use-nvidia"></a>

## How to use nvidia

When launching the developer script you can use the options `-g` to use the integrated graphics card or `-n` to use the nvidia graphics card. Before you start, make sure you have the [NVIDIA Container Toolkit installed](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

Now we will have to install the nvidia runtime to use it with our docker:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-docker2
```

Now we will check if docker recognises nvidia as a new runtime (restarting the docker service to update the new configuration):

```bash
sudo systemctl restart docker
docker info | grep -i runtime
```

It will most likely not recognise it, so we will have to do it manually by editing or creating the `/etc/docker/daemon.json` file:

```json
{
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  }
}
```

It is also possible that nvidia-runtime is not installed, check and install it if it is not.

```bash
dpkg -l | grep nvidia-container-runtime
```

If it is not installed:

```bash
sudo apt-get install -y nvidia-container-runtime
```

Now everything should be ready to start using nvidia with our dockers, restart the docker service to update the configuration and check that everything works correctly.

```bash
sudo systemctl restart docker
```

<a name="How-to-add-a-new-exercise"></a>

## How to add a new exercise

To create a new exercise you must complete this 2 sections:

### Create the Exercise Folder with the source code and frontend

Create a folder with the folder name as "exercise_id" at the location from repository root : "exercises".

Inside that folder create 2 or more new ones with the following names:

- `<language>_template`: the available languages are `python` and `cpp`. One for each supported language.
- `frontend`

You may add a teaser to the exercise by creating a file called `teaser.png` using a 9/10 aspect ratio.

#### Source code: inside `python_template`

An exercise must contain this 3 files:

- **WebGUI.py**: used for the interactions between exercise and frontend.
- **HAL.py**: Hardware Abstraction Layer for accesing the robot data.
- **Frequency.py**: needed for frequency control of iterative code.

There are a three python packages to help the development of a new exercise:

- [Hal Interfaces][]: provides the hardware abstraction layer for various components
- [Gui Interfaces][]: provides with various base GUI's for easy development
- [Console Interfaces][]: provides control of the console

[Hal Interfaces]: ../common/hal_interfaces/README.md
[Gui Interfaces]: ../common/gui_interfaces/README.md
[Console Interfaces]: ../common/console_interfaces/README.md

For knowing how to use each package, please follow the links in the list above.

#### Source code: inside `cpp_template`

**WORK IN PROGRESS**

#### Frontend: inside `frontend`

An exercise must contain the following filee:

- **tsconfig.json**: used for the tsconfig. **Must** contain the next code:

```json
{
  "compilerOptions": {
    "target": "es5",
    "lib": ["dom", "dom.iterable", "esnext"],
    "types": [
      "../../../react_frontend/svg.d.ts",
      "../../../react_frontend/png.d.ts",
      "../../../react_frontend/jpg.d.ts",
      "../../../react_frontend/zip.d.ts"
    ],
    "paths": {
      "Assets/*": ["../../../react_frontend/src/assets/*"],
      "Components/*": ["../../../react_frontend/src/components/*"],
      "Utils/*": ["../../../react_frontend/src/utils/*"],
      "Contexts/*": ["../../../react_frontend/src/contexts/*"],
      "Helpers/*": ["../../../react_frontend/src/helpers/*"],
      "Icons/*": ["../../../react_frontend/src/icons/*"],
      "Styles/*": ["../../../react_frontend/src/styles/*"],
      "Types/*": ["../../../react_frontend/src/types/*"],
      "Constants/*": ["../../../react_frontend/src/constants/*"],
      "Api": ["../../../react_frontend/src/api/index.ts"],
      "Routes": ["../../../react_frontend/src/routes/index.ts"],
      "*": ["../../../react_frontend/node_modules/*"]
    },
    "baseUrl": "./",
    "rootDirs": ["./", "../../../react_frontend/src"],
    "typeRoots": ["../../../react_frontend/node_modules/@types"],
    "declaration": true,
    "allowJs": true,
    "skipLibCheck": true,
    "esModuleInterop": true,
    "allowSyntheticDefaultImports": true,
    "strict": true,
    "noImplicitAny": true,
    "forceConsistentCasingInFileNames": true,
    "noFallthroughCasesInSwitch": true,
    "module": "esnext",
    "moduleResolution": "node",
    "resolveJsonModule": true,
    "isolatedModules": true,
    "noEmit": true,
    "jsx": "react-jsx"
  },
  "include": [
    "./",
    "eslint.config.mts",
    "../../../react_frontend/src",
    "../../../react_frontend/png.d.ts",
    "../../../react_frontend/jpg.d.ts",
    "../../../react_frontend/svg.d.ts",
    "../../../react_frontend/zip.d.ts"
  ]
}
```

- **eslint.config.mts**: used for the eslint (**Currently does not work**). **Must** contain the next code:

```typescript
import js from "@eslint/js";
import globals from "globals";
import tseslint from "typescript-eslint";
import pluginReact from "eslint-plugin-react";
import { defineConfig } from "eslint/config";

export default defineConfig([
  {
    files: ["**/*.{js,mjs,cjs,ts,mts,cts,jsx,tsx}"],
    plugins: { js },
    extends: ["js/recommended"],
    languageOptions: {
      globals: globals.browser,
      parserOptions: {
        project: "./tsconfig.json",
        tsconfigRootDir: import.meta.dirname,
      },
    },
  },
  tseslint.configs.recommended,
  pluginReact.configs.flat.recommended,
  {
    settings: {
      react: {
        version: "detect",
      },
    },
  },
]);
```

- **WebGUI.tsx**: used for the exercise frontend.

And should have the following:

- **updateCallback**: will be called when the data is sent from the application. In here you must update the shown data. **Obligatory**
- **stateCallback**: will be called when the state of the Robotics Backend is changed. In here you may unset shown images or reset variables when the application is reset.
- **canvasRef** and **resizeObserver**: should be used when information displayed is dependant on the screen size. **resizeObserver** will be called when the size changes.

```typescript
import React, { useState, useEffect } from "react";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";
import { useExercise } from "Contexts/ExerciseContext";
import { states } from "jderobot-commsmanager";

function WebGUI() {
  const exerciseContext = useExercise();
  const [image, setImage] = useState<string | undefined>(undefined);
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    const data = updateData as any;
    const update = data.update;

    if (update.image) {
      const image = JSON.parse(update.image);
      setImage(`data:image/png;base64,${image.image}`);
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setImage(undefined);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      <WebGUIImage id="gui_canvas" src={image} style={{ width: "100%" }} />
    </WebGUIContainer>
  );
}

export default WebGUI;
```

You may add as many TypeScript helper files needed inside a helper directory, and as many CSS files inside a css folder.

If there is need for additional resources such as images, you may add them inside a resources folder.

### Add the exercise to the database

To add a exercise to the database you **must** use Django Web Admin:

1.  Launch the docker as normal.
2.  Access http://127.0.0.1:7164/admin/ on a browser and log in with "user" and "pass".
3.  Click on "add exercise" and fill the required fields specified below.
4.  Save the exercise clicking on the "Local Save" button.

An exercise entry in the database must include the following data:

- `exercise id`: unique exercise identifier, must match the folder name
- `name`: name to display on the exercise list
- `description`: description to display on the exercise list
- `tags`: an exercise must include at least one ROS tag ("ROS2"). The exercise will only be shown on the exercise list when the RoboticsBackend ROS version installed is listed in the tags. Tags are also used by the search bar.
- `status`: changes the state indicator (ACTIVE = green; PROTOTYPE = yellow; INACTIVE = red)
- `url`: url of the exercise documentation

<a name="Steps-to-change-models-from-CustomRobots-in-RoboticsAcademy-exercises"></a>

## Steps to change models from CustomRobots in RoboticsAcademy exercises.

- Follow the guide in [Robotics Infrastructure](https://github.com/JdeRobot/RoboticsInfrastructure).

<!-- <a name="edit-code-on-RoboticsBackend-on-the-go"></a>

## Edit code on RoboticsBackend On The GO.

1. If your IDE of choice is VSCode then this method is for you, visit [Remote Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) to download the extenstion.

2. Start the Robotics Academy Docker Image

3. Start VS Code

4. Run the Remote-Containers: Open Folder in Container... command and select the local folder.

   <img width="597" alt="remote-command-palette" src="https://user-images.githubusercontent.com/58532023/184609609-eb1c1a15-9666-46f9-bc9d-df099d3738b8.png">

## How to add your local changes to RoboticsBackend while persisting changes two-way

1. This method is for you if you have worked your way till now in your local setup and looking to import all changes inside RoboticsBackend while also being able to edit and persist further changes.

2. On Terminal open the directory where your project or code is located at (Example:- `cd ~/my_project`)

3. Append `-v $(pwd):/location_in_radi` to your `docker run` cli command used to run your container. (Example:- `docker run --rm -it $(nvidia-smi >/dev/null 2>&1 && echo "--gpus all" || echo "") --device /dev/dri -p 7164:7164 -p 6080-6090:6080-6090 -p 7163:7163 jderobot/robotics-backend -v $(pwd):/home jderobot/robotics-academy`)

4. This will import your local directory inside the docker container, if you have used the example command like above where the location the command is being run is mounted to the home folder inside the docker container you will simply be able to see all the local mounted directories inside the /home of the RoboticsBackend.

5. To make sure that your local directory has been mounted correctly to the correct location inside RoboticsBackend, navigate to http://localhost:1108/vnc.html after launching an exercise(This involves clicking on the launch button of any exercise of your choice) and this will open an vnc console Instance where you may verify the integrity of the mount.

   ![Screenshot from 2022-08-22 01-31-16](https://user-images.githubusercontent.com/58532023/185808802-3a207cb5-b2df-466f-a7f1-70864ff34206.png) -->
