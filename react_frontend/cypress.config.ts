import { defineConfig } from "cypress";

export default defineConfig({
  projectId: "3v23eo",
  allowCypressEnv: false,

  e2e: {
    setupNodeEvents(on, config) {
      // implement node event listeners here
    },
    baseUrl: "http://127.0.0.1:7164",
  },
});
