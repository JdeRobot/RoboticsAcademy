/* eslint-disable no-undef */
/* eslint-disable @typescript-eslint/no-require-imports */
const MiniCssExtractPlugin = require("mini-css-extract-plugin");
const BundleTrackerPlugin = require("webpack-bundle-tracker");
const path = require("path");

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
    Icons: path.resolve(__dirname, "src/icons"),
    Styles: path.resolve(__dirname, "src/styles"),
    Types: path.resolve(__dirname, "src/types"),
    Constants: path.resolve(__dirname, "src/constants"),
    Routes: path.resolve(__dirname, "src/routes"),
    Api: path.resolve(__dirname, "src/api"),
    Common: path.resolve(__dirname, "../static/common"),
    exercises: path.resolve(__dirname, "../exercises/static/exercises"),
  };

  return aliasConfig;
};

module.exports = {
  entry: {
    main: "./src/index.tsx",
  },
  output: {
    filename: "js/[name].[contenthash:8].js",
    clean: true,
  },
  resolve: {
    alias: aliases(),
    extensions: [".js", ".jsx", ".ts", ".tsx", ".json"],
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
        test: /\.(js|jsx|ts|tsx)$/,
        exclude: /node_modules/,
        use: [
          {
            loader: "babel-loader",
            options: {
              presets: [
                "@babel/preset-env",
                ["@babel/preset-react", { runtime: "automatic" }],
                "@babel/preset-typescript",
              ],
            },
          },
        ],
      },
      {
        test: /\.(png|jpg|gif)$/,
        type: "asset/resource",
      },
      {
        test: /\.svg$/i,
        use: [
          {
            loader: "@svgr/webpack",
            options: {
              svgoConfig: {
                plugins: [{ name: "preset-default", removeViewBox: false }],
              },
            },
          },
          "file-loader",
        ],
      },
      {
        test: /\.(zip)$/,
        use: "binary-loader",
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
