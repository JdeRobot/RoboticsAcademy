/// <reference types="cypress" />

context('Themes', () => {
  beforeEach(() => {
    cy.visit('http://127.0.0.1:7164/academy/')
  })

  it('Switch Theme', () => {
    cy.get('#theme-button').click()
    cy.get('#theme-button').children().filter('[data-testid=DarkModeRoundedIcon]')
    // cy.get('#theme-button').children().should('have.data-testid', "DarkModeRoundedIcon")
  })
})
