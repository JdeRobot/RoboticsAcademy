/** @type {import('tailwindcss').Config} */
export default {
  content: [
    // "./src/App.js"
    "./templates/base.html",
    "./src/*.{html,js,ts,jsx,tsx}",
    "./src/**/*.{html,js,ts,jsx,tsx}",
    "./src/index.js",
  ],
  theme: {
    extend: {},
  },
  plugins: [],
};
