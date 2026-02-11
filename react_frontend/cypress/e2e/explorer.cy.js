/// <reference types="cypress" />

context("Themes", () => {
  beforeEach(() => {
    cy.visit("/academy/");
    cy.viewport("macbook-16");
    cy.get(`#follow_line`).click();
    cy.contains("academy.py", { timeout: 1000 });
  });

  afterEach(() => {
    cy.get("#return-academy").click();
  });

  it("File in root", () => {
    cy.get("#new-file-button").click();
    cy.get("#new-file-modal")
      .find("form")
      .within(() => {
        cy.get("input").type("root.txt", { force: true });
        cy.contains("button", "Create").click({ force: true });
      });
    cy.wait(1000);
    cy.contains("root.txt", { timeout: 1000 }).click();

    // cy.get("#rename-file-button").click();
    // cy.get("#renameData")
    //   .clear({ force: true })
    //   .type("root2.txt", { force: true });
    // cy.get("#create-new-action", { timeout: 1000 }).click({ force: true });
    // cy.wait(1000);
    // cy.contains("root2.txt", { timeout: 1000 }).click();

    cy.get("#delete-file-button").click();
    cy.get("#delete-selected-button", { timeout: 1000 }).click({ force: true });
    cy.wait(1000);
  });

  it("Folder in root", () => {
    cy.get("#new-folder-button").click();
    cy.get("#folderName", { timeout: 1000 }).type("root", { force: true });
    cy.get("#create-new-action").click({ force: true });
    cy.wait(1000);
    cy.contains("root", { timeout: 1000 }).click();

    // cy.get("#rename-file-button").click();
    // cy.get("#renameData", { timeout: 1000 }).should("contains.text", "root");
    // cy.get("#renameData").type("root2");
    // cy.get("#create-new-action").click();
    // cy.wait(1000);
    // cy.contains("root2", { timeout: 1000 }).click();

    cy.contains("root", { timeout: 1000 })
      .get("#explorer-action-button", { force: true })
      .click({ force: true });

    cy.contains("label", "Delete", { force: true })
      .parent({ force: true })
      .click({ force: true });

    cy.get("#delete-selected-button", { timeout: 1000 }).click({ force: true });
    cy.wait(1000);
  });

  it("File in folder", () => {
    cy.get("#new-folder-button").click();
    cy.get("#folderName", { timeout: 1000 }).type("root", { force: true });
    cy.get("#create-new-action").click({ force: true });
    cy.wait(1000);
    cy.contains("root", { timeout: 1000 }).click();
    cy.contains("root", { timeout: 1000 }).click();

    cy.get("#new-file-button").click();
    cy.get("#fileName", { timeout: 1000 }).type("inside.txt", { force: true });
    cy.get("#create-new-file").click({ force: true });
    cy.wait(1000);
    cy.contains("inside.txt", { timeout: 1000 }).click();

    // cy.get("#rename-file-button").click();
    // cy.get("#renameData", { timeout: 1000 }).should(
    //   "contains.text",
    //   "inside.txt"
    // );
    // cy.get("#renameData").type("inside2.txt");
    // cy.get("#create-new-action").click();
    // cy.wait(1000);
    // cy.contains("inside2.txt", { timeout: 1000 }).click();

    cy.get("#delete-file-button").click();
    cy.get("#delete-selected-button", { timeout: 1000 }).click({ force: true });
    cy.wait(1000);

    cy.contains("root", { timeout: 1000 })
      .get("#explorer-action-button", { force: true })
      .click({ force: true });

    cy.contains("label", "Delete", { force: true })
      .parent({ force: true })
      .click({ force: true });

    cy.get("#delete-selected-button", { timeout: 1000 }).click({ force: true });
    cy.wait(1000);
  });

  it("Folder in folder", () => {
    cy.get("#new-folder-button").click();
    cy.get("#folderName", { timeout: 1000 }).type("root", { force: true });
    cy.get("#create-new-action").click({ force: true });
    cy.wait(1000);
    cy.contains("root", { timeout: 1000 }).click();
    cy.contains("root", { timeout: 1000 }).click();

    cy.get("#new-folder-button").click();
    cy.get("#folderName", { timeout: 1000 }).type("inside", { force: true });
    cy.get("#create-new-action").click({ force: true });
    cy.wait(1000);

    // cy.get("#rename-file-button").click();
    // cy.get("#renameData", { timeout: 1000 }).should("contains.text", "inside");
    // cy.get("#renameData").type("inside2");
    // cy.get("#create-new-action").click();
    // cy.wait(1000);
    // cy.contains("inside2", { timeout: 1000 }).click();

    cy.contains("inside", { timeout: 1000 })
      .get("#explorer-action-button", { force: true })
      .click({ force: true });

    cy.contains("label", "Delete", { force: true })
      .parent({ force: true })
      .click({ force: true });

    cy.get("#delete-selected-button", { timeout: 1000 }).click({ force: true });
    cy.wait(1000);

    cy.contains("root", { timeout: 1000 })
      .get("#explorer-action-button", { force: true })
      .click({ force: true });

    cy.contains("label", "Delete", { force: true })
      .parent({ force: true })
      .click({ force: true });

    cy.get("#delete-selected-button", { timeout: 1000 }).click({ force: true });
    cy.wait(1000);
  });
});
