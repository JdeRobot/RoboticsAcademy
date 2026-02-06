/// <reference types="cypress" />

context('Loading', () => {
  it('Load Home', () => {
    cy.visit('http://127.0.0.1:7164/academy/')
  })
})
