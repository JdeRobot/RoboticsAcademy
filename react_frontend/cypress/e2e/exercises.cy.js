/// <reference types="cypress" />

// const sizes = ["macbook-11", "macbook-13", "macbook-15", "macbook-16"];
const sizes = ["macbook-16"];
const exercises2 = [
  {
    id: "follow_line",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/follow_line/",
  },
  {
    id: "vacuum_cleaner",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/vacuum_cleaner",
  },
  {
    id: "autoparking",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/autoparking",
  },
  {
    id: "follow_person",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/follow_person",
  },
  {
    id: "vacuum_cleaner_loc",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/vacuum_cleaner_loc",
  },
  {
    id: "global_navigation",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/global_navigation",
  },
  {
    id: "rescue_people",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/Drones/rescue_people",
  },
  {
    id: "obstacle_avoidance",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/obstacle_avoidance",
  },
  {
    id: "3d_reconstruction",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/3d_reconstruction",
  },
  {
    id: "amazon_warehouse",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/amazon_warehouse",
  },
  {
    id: "montecarlo_laser_loc",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/montecarlo_laser_loc",
  },
  {
    id: "montecarlo_visual_loc",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/montecarlo_visual_loc",
  },
  {
    id: "marker_visual_loc",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/marker_visual_loc",
  },
  {
    id: "laser_mapping",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/MobileRobots/laser_mapping",
  },
  {
    id: "basic_computer_vision",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/basic_computer_vision",
  },
  {
    id: "follow_road",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/Drones/follow_road",
  },
  {
    id: "pick_place",
    webGui: false,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/IndustrialRobots/pick_place",
  },
  {
    id: "image_classification",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/image_classification",
  },
  {
    id: "object_detection",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/object_detection",
  },
  {
    id: "drone_gymkhana",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/Drones/drone_gymkhana",
  },
  {
    id: "power_tower_inspection",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/Drones/power_tower_inspection",
  },
  {
    id: "end_to_end_visual_control",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/end_to_end_visual_control",
  },
  {
    id: "digital_image_processing",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/digital_image_processing",
  },
  {
    id: "car_junction",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/car_junction",
  },
  {
    id: "machine_vision",
    webGui: false,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/IndustrialRobots/machine_vision",
  },
  {
    id: "labyrinth_escape",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/Drones/labyrinth_escape",
  },
];
const exercises = [
  {
    id: "follow_line",
    webGui: true,
    docs: "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/follow_line/",
  },
];

exercises2.forEach((exercise) => {
  context(`Exercise: ${exercise.id}`, () => {
    before(() => {
      cy.visit("http://127.0.0.1:7164/academy/");
      // Cypress.on("uncaught:exception", (err, runnable) => {
      //   // returning false here prevents Cypress from
      //   // failing the test
      //   return false;
      // });
    });

    after(() => {
      cy.get("#return-academy").click();
    });

    sizes.forEach((size) => {
      it(`Exercise`, () => {
        cy.viewport(size);
        cy.get(`#${exercise.id}`).click();
        cy.contains("academy.py", { timeout: 1000 });
        if (exercise.webGui) {
          cy.get(`#webgui-container`, { timeout: 1000 });
        }

        // Test docs
        cy.window().then((win) => {
          cy.stub(win, "open").as("windowOpen");
        });
        cy.get("#theory-button").click();
        cy.get("@windowOpen").should("have.been.called");

        cy.get("@windowOpen")
          .its("lastCall.args.0")
          .then((newWindowUrl) => {
            expect(newWindowUrl.href).to.equal(exercise.docs);
          });

        cy.get("#info-button").click();
        cy.get("@windowOpen").should("have.been.called");

        cy.get("@windowOpen")
          .its("lastCall.args.0")
          .then((newWindowUrl) => {
            expect(newWindowUrl.href).to.equal(
              "https://jderobot.github.io/RoboticsAcademy/user_guide/"
            );
          });

        cy.get("#forum-button").click();
        cy.get("@windowOpen").should("have.been.called");

        cy.get("@windowOpen")
          .its("lastCall.args.0")
          .then((newWindowUrl) => {
            expect(newWindowUrl.href).to.equal(
              "https://github.com/JdeRobot/RoboticsAcademy/discussions"
            );
          });

        cy.get("#connect-with-rb").click();
        cy.get("#robotics-backend-state", { timeout: 2000 }).contains(
          "tools_ready",
          { timeout: 15000 }
        );
      });
    });
  });
});
