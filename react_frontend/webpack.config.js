const MiniCssExtractPlugin = require("mini-css-extract-plugin");
const BundleTrackerPlugin = require("webpack-bundle-tracker");

module.exports = {
  entry: "./src/index.js",
  output: {
    filename: "js/[name].[hash:8].js",
    clean: true
  },
  module: {
    rules: [
      {
        test: /\.(s*)css$/,
        use: [
          { loader: MiniCssExtractPlugin.loader },
          "css-loader",
          "sass-loader"
        ]
      },
      {
        test: /\.(js|jsx)$/,
        exclude: /node_modules/,
        use: {
          loader: "babel-loader"
        }
      }
    ]
  },
  plugins: [
      new BundleTrackerPlugin({
        filename: './webpack-stats.json'
      }),
      new MiniCssExtractPlugin({
        filename: 'css/main.css'
      })
  ],
  devtool: 'inline-source-map'
};