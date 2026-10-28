/// <reference types="cypress" />

describe("Run recovers from failed project preparation", () => {
  beforeEach(() => {
    cy.visit("/academy/");
    cy.get("#follow_line").click();
    cy.contains("academy.py", { timeout: 30000 });
    cy.get("#connect-with-rb").click();
    cy.get("#robotics-backend-state", { timeout: 60000 }).should(
      "contain",
      "tools_ready",
    );
  });

  afterEach(() => {
    cy.get("#stop-world").click();
    cy.get("#robotics-backend-state", { timeout: 30000 }).should(
      "contain",
      "connected",
    );
    cy.intercept("POST", "**/academy/exit_exercise/").as("exitExercise");
    cy.get("#return-academy").click();
    cy.wait("@exitExercise").its("response.statusCode").should("eq", 200);
  });

  const failures = [
    {
      name: "a file-list HTTP error",
      url: /\/academy\/get_file_list\/?(?:\?|$)/,
      response: {
        statusCode: 503,
        body: { message: "File listing temporarily unavailable" },
      },
      expectedError: "File listing temporarily unavailable",
    },
    {
      name: "a malformed file-list response",
      url: /\/academy\/get_file_list\/?(?:\?|$)/,
      response: { statusCode: 200, body: { file_list: "{invalid-json" } },
      expectedError: "Error running app:",
    },
    {
      name: "a source-file HTTP error",
      url: /\/academy\/get_file\/?(?:\?|$)/,
      response: {
        statusCode: 503,
        body: { message: "Source file temporarily unavailable" },
      },
      expectedError: "Source file temporarily unavailable",
    },
  ];

  failures.forEach(({ name, url, response, expectedError }) => {
    it(`reports ${name} and permits a successful retry`, () => {
      cy.intercept({ method: "GET", url, times: 1 }, response).as(
        "preparationFailure",
      );
      cy.get("#run-app").click();
      cy.wait("@preparationFailure");
      cy.contains(expectedError).should("be.visible");
      cy.get("#run-app").should("not.be.disabled");
      cy.get("#robotics-backend-state").should("contain", "tools_ready");
      cy.get("#error-modal").contains("button", "Close").click();
      cy.get("#error-modal").should("not.be.visible");

      // The intercepted failure is exhausted: retry against the real backend.
      cy.get("#run-app").click();
      cy.get("#robotics-backend-state", { timeout: 30000 }).should(
        "contain",
        "application_running",
      );
      cy.get("#run-app").should("not.be.disabled").click();
      cy.get("#robotics-backend-state", { timeout: 30000 }).should(
        "contain",
        "paused",
      );
    });
  });
});
