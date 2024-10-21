import React, { useEffect, useState } from "react";

// monaco editor loader
export const theme_colors = {
  vs: {
    bg: "#fff",
    text: "#333",
    line: "#ccc",
  },
  vs_dark: {
    bg: "#1E1E1E", //rgb(30,30,30)
    text: "#fefefe",
    line: "#111",
  },
  hc_black: {
    bg: "#000",
    text: "#fff",
    line: "#1E1E1E",
  },
};

const lineTheme = [
  {
    width: `15%`,
    height: `16px`,
    marginBottom: ``,
    lineMaringLeft: `0px`,
  },
  {
    width: `20%`,
    height: `16px`,
    marginBottom: ``,
    lineMaringLeft: `0px`,
  },

  {
    width: `0%`,
    height: `0%`,
    marginBottom: `16px`,
    lineMaringLeft: `0px`,
  },
  {
    width: `12%`,
    height: `12px`,
    marginBottom: ``,
    lineMaringLeft: `0px`,
  },

  {
    width: `24%`,
    height: `14px`,
    marginBottom: ``,
    lineMaringLeft: `40px`,
  },

  {
    width: `30%`,
    height: `14px`,
    marginBottom: `12px`,
    lineMaringLeft: `40px`,
  },

  {
    width: `13%`,
    height: `14px`,
    marginBottom: ``,
    lineMaringLeft: `40px`,
  },
  {
    width: `0%`,
    height: `0px`,
    marginBottom: ``,
    lineMaringLeft: ``,
  },
  {
    width: `50%`,
    height: `12px`,
    marginBottom: ``,
    lineMaringLeft: `40px`,
  },
  {
    width: `0%`,
    height: `0px`,
    marginBottom: ``,
    lineMaringLeft: ``,
  },
  {
    width: `0%`,
    height: `0px`,
    marginBottom: ``,
    lineMaringLeft: ``,
  },
  {
    width: `20%`,
    height: `10px`,
    marginBottom: ``,
    lineMaringLeft: ``,
  },
];

const MonacoEditorLoader = ({ theme }) => {
  const [background, setBackground] = useState(null);
  const [lineBackground, setLineBackground] = useState(null);

  useEffect(() => {
    const t = theme_colors[theme.split("-").join("_")];
    setBackground(`${t.bg}`);
    setLineBackground(`${t.line}`);
  }, [theme]);
  return (
    <div
      className={`w-full h-full px-6 py-2 select-none`}
      style={{ backgroundColor: `${background}` }}
    >
      <div
        className={`flex flex-col items-start p-4 animate-pulse w-full h-full `}
      >
        {lineTheme.map((line, i) => (
          <div
            className="flex items-center justify-start gap-4  w-full h-[32px]"
            key={i}
          >
            <span
              className={`text-base`}
              style={{ color: `${lineBackground}` }}
            >
              {i + 1}
            </span>
            <div
              className={`w-2/3 h-3 rounded-full `}
              style={{
                backgroundColor: `${lineBackground}`,
                width: `${line.width}`,
                height: `10px`,
                marginBottom: `${line.marginBottom}`,
                marginLeft: `${line.lineMaringLeft}`,
              }}
            ></div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default MonacoEditorLoader;
