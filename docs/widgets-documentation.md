### React Widgets Documentation

#### Basic Widgets
- Buttons
  - Console Button : This button's purpose is to open/close console viewer widget.
  - Gazebo Button : This button's purpose is to open/close gazebo viewer widget.
  - Load File Button : This button's purpose is to load python file into ace editor widget.
  - Load Into Robot Button : This button loads the code from ace editor into the robot.
  - Play/Stop Button : This button's purpose is to start/stop the exercise robot or task.
  - Reset Button : This button resets the robots to its initial state.
  - Save Button : This button saves the code snippet from ace editor into local computer.
  - TeleOp Button : This button is used to change the position of robot. 
- Common
  - Custom Alert : This widget handles all the four type of alerts depending upon the alert state.
    - Success Alert
    - Informative Alert 
    - Warning Alert
    - Failure Alert
  - Exercise Control : This widget incorporates all the controls that are provided in an exercise with two sliders to adjust GUI frequency and Brain frequency. 
  - Frequency Menu : This widget represents the actual Brain frequency, GUI frequency and Simulation real time factor value.
  - MainAppBar : App bar consists of functions to change view mode, connection to manager and Info Modal.
  - View : This widget consists of three views -
    - Exercise View : Renders the exercise specific view
    - Theory View : Theory related to exercise.
    - Forum View : Forum to discuss issues and share to the community.
  - Visualization Box : This box wraps the visualizers used in an exercise.
- Exercises
  - Ace Editor : Editor to write code 
  - Circuit Selector : Dropdown to select the circuit
  - File Selector : Uploads a file 
  - Gazebo Viewer : Iframe rendering a Gazebo.
  - Console Viewer : Iframe rendering a terminal.
- Modals
  - Error Modal : Modal that pops up when some syntax error, linting error or runtime error is occurred.
  - Info Modal : Modal that displays the information about exercise controls
  - Load Modal : Modal is visible until the code loads into the robot
- Visualizers
  - Bird Eye Canvas : Canvas used for *Bird Eye* Visuals 
  - Three Js Canvas : Canvas to load the *3D Scene*
  - GUI Canvas : Canvas to load the image and draw the movements of robot
  - Image Canvas : Image from Backend is displayed 
  - Local Map Canvas : Canvas for some specific exercise


### Views and Template Widgets

- #### Template
  - Major component that renders the exercise
  - Defines exercise property
  - Sets Exercise Context
  - Basic Structure is as follows -
      ```angular2html
      function ExerciseName() {
      return (
        <Box>
          <ViewProvider>
            <ExerciseProvider>
              <MainAppBar
                exerciseName={" $exercise Name "}
                context={ExerciseContext}
              />
              <View
                url={THEORY_URL.ExerciseName}
                exerciseId={
                  <ExerciseView context={ExerciseContext} />
                }
              />
            </ExerciseProvider>
          </ViewProvider>
        </Box>
        );
      }
      export default ExerciseName;
      ```
  
- #### Views
  - Handles the UI and responsiveness of the exercise.
  - Widgets are defined here that will be displayed for the exercise
  
