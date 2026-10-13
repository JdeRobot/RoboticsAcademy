/// <reference types="cypress" />

// const sizes = ["macbook-11", "macbook-13", "macbook-15", "macbook-16"];
const sizes = ["macbook-16"];

const exercises = [
  {
    id: "follow_line",
  },
  {
    id: "vacuum_cleaner",
  },
  {
    id: "autoparking",
  },
  {
    id: "follow_person",
  },
  {
    id: "vacuum_cleaner_loc",
  },
  {
    id: "global_navigation",
  },
  {
    id: "rescue_people",
  },
  {
    id: "obstacle_avoidance",
  },
  {
    id: "3d_reconstruction",
  },
  {
    id: "amazon_warehouse",
  },
  {
    id: "montecarlo_laser_loc",
  },
  {
    id: "montecarlo_visual_loc",
  },
  {
    id: "marker_visual_loc",
  },
  {
    id: "laser_mapping",
  },
  {
    id: "basic_computer_vision",
  },
  {
    id: "follow_road",
  },
  {
    id: "pick_place",
  },
  {
    id: "image_classification",
  },
  {
    id: "object_detection",
  },
  {
    id: "drone_gymkhana",
  },
  {
    id: "power_tower_inspection",
  },
  {
    id: "end_to_end_visual_control",
  },
  {
    id: "digital_image_processing",
  },
  {
    id: "car_junction",
  },
  {
    id: "machine_vision",
  },
  {
    id: "labyrinth_escape",
  },
];

exercises.forEach((exercise) => {
  context(`Exercise: ${exercise.id}`, () => {
    before(() => {
      cy.visit("http://127.0.0.1:7164/academy/");
    });

    after(() => {
      cy.get("#return-academy").click();
    });

    sizes.forEach((size) => {
      it(`Execution`, () => {
        cy.viewport(size);
        cy.get(`#${exercise.id}`).click();
        cy.contains("academy.py", { timeout: 1000 });
        if (exercise.webGui) {
          cy.get(`#webgui-container`, { timeout: 1000 });
        }

        cy.get("#connect-with-rb").click();
        cy.get("#robotics-backend-state", { timeout: 2000 }).contains(
          "world_ready",
          { timeout: 15000 }
        );
        cy.get("#robotics-backend-state").contains("tools_ready", {
          timeout: 15000,
        });

        cy.get("#run-app").click();
        cy.get("#robotics-backend-state").contains("application_running", {
          timeout: 15000,
        });

        cy.screenshot();
        cy.wait(1000);

        cy.get("#run-app").click();
        cy.get("#robotics-backend-state").contains("paused", {
          timeout: 15000,
        });

        cy.get("#reset-app").click();
        cy.get("#robotics-backend-state").contains("tools_ready", {
          timeout: 15000,
        });

        cy.get("#world-selector")
          .children()
          .then(($body) => {
            if ($body.text().includes("No worlds to select")) {
              // No worlds
            } else {
              cy.get("#stop-world").click();
              cy.get("#robotics-backend-state").contains("connected", {
                timeout: 15000,
              });
            }
          });
      });
    });
  });
});
