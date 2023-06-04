const MiniCssExtractPlugin = require("mini-css-extract-plugin");
const BundleTrackerPlugin = require("webpack-bundle-tracker");
const path = require("path");
const glob = require("glob");

const aliases = () => {
  const exercises_folder =
    "../exercises/static/exercises/**/react-components/";
  const exercises = glob.sync(path.resolve(__dirname, exercises_folder));
  const aliasConfig = {
    _: [
      "./src/components",
      "./src/components/exercises",
      "./src/components/contexts",
    ],
    exercise: [],
  };

  exercises.map((exercise) => {
    const exercisePath = exercise.split(path.sep);
    const exerciseName = exercisePath[exercisePath.length - 3];
    aliasConfig[`exercise`].push(exercise);
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
