/// <reference types="cypress" />

// const sizes = ["macbook-11", "macbook-13", "macbook-15", "macbook-16"];
const sizes = ["macbook-16"];
const exercises2 = [
  { id: "follow_line", webGui: true },
  { id: "vacuum_cleaner", webGui: true },
  { id: "autoparking", webGui: true },
  { id: "follow_person", webGui: true },
  { id: "vacuum_cleaner_loc", webGui: true },
  { id: "global_navigation", webGui: true },
  { id: "rescue_people", webGui: true },
  { id: "obstacle_avoidance", webGui: true },
  { id: "3d_reconstruction", webGui: true },
  { id: "amazon_warehouse", webGui: true },
  { id: "montecarlo_laser_loc", webGui: true },
  { id: "montecarlo_visual_loc", webGui: true },
  { id: "marker_visual_loc", webGui: true },
  { id: "laser_mapping", webGui: true },
  { id: "basic_computer_vision", webGui: true },
  { id: "follow_road", webGui: true },
  { id: "pick_place", webGui: false },
  { id: "image_classification", webGui: true },
  { id: "object_detection", webGui: true },
  { id: "drone_gymkhana", webGui: true },
  { id: "power_tower_inspection", webGui: true },
  { id: "end_to_end_visual_control", webGui: true },
  { id: "digital_image_processing", webGui: true },
  { id: "car_junction", webGui: true },
  { id: "machine_vision", webGui: false },
  { id: "labyrinth_escape", webGui: true },
];
const exercises = [{ id: "follow_line", webGui: true }];

context("Exercises", () => {
  beforeEach(() => {
    cy.visit("http://127.0.0.1:7164/academy/");
    // Cypress.on("uncaught:exception", (err, runnable) => {
    //   // returning false here prevents Cypress from
    //   // failing the test
    //   return false;
    // });
  });

  afterEach(() => {
    cy.get("#return-academy").click();
  });

  sizes.forEach((size) => {
    exercises.forEach((exercise) => {
      it(`Exercise: ${exercise.id}`, () => {
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
            expect(newWindowUrl.href).to.equal(
              "https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/follow_line/"
            );
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

        // cy.get("#connect-with-rb").click();
        // cy.get("#robotics-backend-state", { timeout: 2000 }).contains(
        //   "tools_ready",
        //   { timeout: 10000 }
        // );
      });
    });
  });
});
