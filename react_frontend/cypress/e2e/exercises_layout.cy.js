/// <reference types="cypress" />

// tools: ["WebGUI", "Simulator", "Console", "Webcam","Rviz","Video"]
const exercises = [
  {
    id: "follow_line",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/follow_line/",
  },
  {
    id: "vacuum_cleaner",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/vacuum_cleaner",
  },
  {
    id: "autoparking",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/autoparking",
  },
  {
    id: "follow_person",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/follow_person",
  },
  {
    id: "vacuum_cleaner_loc",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/vacuum_cleaner_loc",
  },
  {
    id: "global_navigation",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation",
  },
  {
    id: "rescue_people",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/Drones/rescue_people",
  },
  {
    id: "obstacle_avoidance",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/obstacle_avoidance",
  },
  {
    id: "3d_reconstruction",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/3d_reconstruction",
  },
  {
    id: "amazon_warehouse",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/amazon_warehouse",
  },
  {
    id: "montecarlo_laser_loc",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/montecarlo_laser_loc",
  },
  {
    id: "montecarlo_visual_loc",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/montecarlo_visual_loc",
  },
  {
    id: "marker_visual_loc",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/marker_visual_loc",
  },
  {
    id: "laser_mapping",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/laser_mapping",
  },
  {
    id: "basic_computer_vision",
    tools: ["WebGUI", "Webcam", "Video", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/basic_computer_vision",
  },
  {
    id: "follow_road",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/Drones/follow_road",
  },
  {
    id: "pick_place",
    tools: ["Rviz", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/IndustrialRobots/pick_place",
  },
  {
    id: "image_classification",
    tools: ["WebGUI", "Webcam", "Video", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/image_classification",
  },
  {
    id: "object_detection",
    tools: ["WebGUI", "Webcam", "Video", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/object_detection",
  },
  {
    id: "drone_gymkhana",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/Drones/drone_gymkhana",
  },
  {
    id: "power_tower_inspection",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/Drones/power_tower_inspection",
  },
  {
    id: "end_to_end_visual_control",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/end_to_end_visual_control",
  },
  {
    id: "digital_image_processing",
    tools: ["WebGUI", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/digital_image_processing",
  },
  {
    id: "car_junction",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/car_junction",
  },
  {
    id: "machine_vision",
    tools: ["Rviz", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/IndustrialRobots/machine_vision",
  },
  {
    id: "labyrinth_escape",
    tools: ["WebGUI", "Simulator", "Console"],
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/Drones/labyrinth_escape",
  },
];

exercises.forEach((exercise) => {
  context(`Exercise: ${exercise.id}`, () => {
    beforeEach(() => {
      cy.visit("/academy/");
      cy.window().then((win) => {
        cy.stub(win, "open").as("windowOpen");
      });
      cy.viewport("macbook-16");
      cy.get(`#${exercise.id}`).click();
    });

    afterEach(() => {
      cy.get("#return-academy").click();
    });

    it(`Explorer`, () => {
      cy.contains("academy.py", { timeout: 1000 });
    });

    if (exercise.tools.includes("WebGUI")) {
      it(`WebGUI`, () => {
        cy.get(`#webgui-container`, { timeout: 1000 });
      });
    }

    it(`Exercise Documentation`, () => {
      cy.get("#theory-button").click();
      cy.get("@windowOpen").should("have.been.called");

      cy.get("@windowOpen")
        .its("lastCall.args.0")
        .then((newWindowUrl) => {
          expect(newWindowUrl.href).to.equal(exercise.docs);
        });
    });

    it(`User Guide`, () => {
      cy.get("#info-button").click();
      cy.get("@windowOpen").should("have.been.called");

      cy.get("@windowOpen")
        .its("lastCall.args.0")
        .then((newWindowUrl) => {
          expect(newWindowUrl.href).to.equal(
            "https://jderobot.github.io/RoboticsAcademy/user_guide/"
          );
        });
    });

    it(`Forum`, () => {
      cy.get("#forum-button").click();
      cy.get("@windowOpen").should("have.been.called");

      cy.get("@windowOpen")
        .its("lastCall.args.0")
        .then((newWindowUrl) => {
          expect(newWindowUrl.href).to.equal(
            "https://github.com/JdeRobot/RoboticsAcademy/discussions"
          );
        });
    });
  });
});
