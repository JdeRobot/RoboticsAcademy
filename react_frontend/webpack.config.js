const MiniCssExtractPlugin = require("mini-css-extract-plugin");
const BundleTrackerPlugin = require("webpack-bundle-tracker");
const path = require("path");
const glob = require("glob");

const aliases = () => {
  const aliasConfig = {
    _: [
      "./src/components",
      "./src/components/exercises",
      "./src/components/contexts",
    ],
    Contexts: path.resolve(__dirname, "src/contexts"),
    Components: path.resolve(__dirname, "src/components"),
    Helpers: path.resolve(__dirname, "src/helpers"),
    Hooks: path.resolve(__dirname, "src/hooks"),
    Common: path.resolve(__dirname, "../static/common"),
    exercises: [],
  };

  const exercises_folder = "../exercises/static/exercises/**/react-components/";
  const exercises = glob.sync(path.resolve(__dirname, exercises_folder));
  exercises.map((exercise) => {
    const exercisePath = exercise.split(path.sep);
    const exerciseName = exercisePath[exercisePath.length - 3];
    aliasConfig[`exercises`].push(exercise);
  });

  return aliasConfig;
};

module.exports = {
  entry: {
    index: "./src/index.js",
    exercise: "./src/exercise-index.js",
  },
  output: {
    filename: "js/[name].[contenthash:8].js",
    clean: true,
  },
  resolve: {
    alias: aliases(),
    modules: ["node_modules", path.resolve(__dirname, "node_modules")],
  },
  module: {
    rules: [
      {
        test: /\.(s*)css$/,
        use: [
          {
            loader: MiniCssExtractPlugin.loader,
          },
          {
            loader: "css-loader",
            options: {
              url: false,
              sourceMap: true,
            },
          },
          "sass-loader",
        ],
      },
      {
        test: /\.(js|jsx)$/,
        exclude: /node_modules/,
        use: [
          {
            loader: "babel-loader",
            options: {
              presets: [
                "@babel/preset-env",
                ["@babel/preset-react", { runtime: "automatic" }],
              ],
            },
          },
        ],
      },
      {
        test: /\.(png|svg|jpg|gif)$/,
        type: "asset/resource",
      },
    ],
  },
  plugins: [
    new BundleTrackerPlugin({
      filename: "./webpack-stats.json",
    }),
    new MiniCssExtractPlugin({
      filename: "css/[name].css",
    }),
  ],
  devtool: "inline-source-map",
};
