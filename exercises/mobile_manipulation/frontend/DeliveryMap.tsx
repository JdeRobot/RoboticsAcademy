import React from "react";

export type DeliveryState = {
  pose: number[] | null;
  path: number[][];
  red_target: number;
  blue_target: number;
  red_score: number;
  blue_score: number;
};

// Guide tape graph of the warehouse delivery world in world coordinates
const NODES: { [name: string]: number[] } = {
  red_dispenser: [6.411, -2.0],
  blue_dispenser: [6.411, -6.0],
  red_turn: [5.4, -2.0],
  dispensers: [5.4, -4.0],
  blue_turn: [5.4, -6.0],
  east: [4.0, -4.0],
  station_5: [4.0, 0.5],
  south: [1.0, -4.0],
  station_3: [1.0, -8.3],
  center: [1.0, 0.0],
  station_4: [1.0, 5.2],
  west: [-3.6, 0.0],
  station_1: [-3.6, 5.7],
  station_2: [-3.6, -3.2],
};

const EDGES = [
  ["red_dispenser", "red_turn"],
  ["red_turn", "dispensers"],
  ["dispensers", "blue_turn"],
  ["blue_turn", "blue_dispenser"],
  ["dispensers", "east"],
  ["east", "station_5"],
  ["east", "south"],
  ["south", "station_3"],
  ["south", "center"],
  ["center", "station_4"],
  ["center", "west"],
  ["west", "station_1"],
  ["west", "station_2"],
];

// Delivery boxes and ball dispensers are drawn a bit past their node so both read clearly
const BOXES: { [station: number]: number[] } = {
  1: [-3.6, 6.5],
  2: [-3.6, -4.0],
  3: [1.0, -9.1],
  4: [1.0, 6.0],
  5: [4.0, 1.3],
};

const CUPS: { [color: string]: number[] } = {
  red: [6.95, -2.0],
  blue: [6.95, -6.0],
};

const WALL_X = 7.3;

const RED = "#e01b1b";
const BLUE = "#1e5bff";
const TAPE = "#94a3b8";
const INK = "#334155";

// The map shows +x up and +y left like the warehouse seen from the dispensers
const toMap = (x: number, y: number) => [-y, -x];

const VIEW = { left: -7.4, top: -7.8, width: 17.2, height: 12.0 };

const DeliveryMap = ({ state }: { state: DeliveryState }) => {
  const targetColor = (station: number) => {
    if (station === state.red_target) {
      return RED;
    }
    if (station === state.blue_target) {
      return BLUE;
    }
    return null;
  };

  const tape = EDGES.map(([a, b]) => {
    const [u1, v1] = toMap(NODES[a][0], NODES[a][1]);
    const [u2, v2] = toMap(NODES[b][0], NODES[b][1]);
    return (
      <line
        key={`${a}-${b}`}
        x1={u1}
        y1={v1}
        x2={u2}
        y2={v2}
        stroke={TAPE}
        strokeWidth={0.12}
        strokeLinecap="round"
      />
    );
  });

  const path = state.path.map((p) => toMap(p[0], p[1]).join(",")).join(" ");

  const nodes = Object.entries(NODES).map(([name, [x, y]]) => {
    const [u, v] = toMap(x, y);
    const station = name.startsWith("station_") ? Number(name.slice(8)) : 0;
    const color = station ? targetColor(station) : null;
    return (
      <circle
        key={name}
        cx={u}
        cy={v}
        r={color ? 0.32 : 0.17}
        fill={color ?? "white"}
        stroke={color ? "white" : INK}
        strokeWidth={color ? 0.08 : 0.06}
      />
    );
  });

  const boxes = Object.entries(BOXES).map(([station, [x, y]]) => {
    const [u, v] = toMap(x, y);
    const color = targetColor(Number(station)) ?? INK;
    return (
      <g key={station}>
        <rect
          x={u - 0.3}
          y={v - 0.3}
          width={0.6}
          height={0.6}
          rx={0.08}
          fill="#f1e3c8"
          stroke={color}
          strokeWidth={0.07}
        />
        <text
          x={u}
          y={v + 0.18}
          textAnchor="middle"
          fontSize={0.5}
          fontWeight="bold"
          fill={color}
        >
          {station}
        </text>
      </g>
    );
  });

  const cups = Object.entries(CUPS).map(([color, [x, y]]) => {
    const [u, v] = toMap(x, y);
    return (
      <rect
        key={color}
        x={u - 0.3}
        y={v - 0.25}
        width={0.6}
        height={0.5}
        rx={0.1}
        fill={color === "red" ? RED : BLUE}
      />
    );
  });

  let robot = null;
  if (state.pose) {
    const [x, y, yaw] = state.pose;
    const [u, v] = toMap(x, y);
    const heading =
      (Math.atan2(-Math.cos(yaw), -Math.sin(yaw)) * 180) / Math.PI;
    robot = (
      <g transform={`translate(${u} ${v}) rotate(${heading})`}>
        <rect
          x={-0.32}
          y={-0.24}
          width={0.64}
          height={0.48}
          rx={0.08}
          fill="#f5a623"
          stroke={INK}
          strokeWidth={0.04}
        />
        <polygon points="0.3,0 0.06,-0.16 0.06,0.16" fill={INK} />
      </g>
    );
  }

  const [, wallV] = toMap(WALL_X, 0);

  return (
    <svg
      viewBox={`${VIEW.left} ${VIEW.top} ${VIEW.width} ${VIEW.height}`}
      preserveAspectRatio="xMidYMid meet"
      style={{ width: "100%", height: "100%", display: "block" }}
    >
      <rect
        x={VIEW.left}
        y={VIEW.top}
        width={VIEW.width}
        height={VIEW.height}
        fill="white"
      />
      <line
        x1={VIEW.left + 0.3}
        y1={wallV}
        x2={VIEW.left + VIEW.width - 0.3}
        y2={wallV}
        stroke={INK}
        strokeWidth={0.1}
      />
      {tape}
      {state.path.length > 1 && (
        <polyline
          points={path}
          fill="none"
          stroke="#22c55e"
          strokeWidth={0.22}
          strokeLinejoin="round"
          strokeLinecap="round"
        />
      )}
      {cups}
      {boxes}
      {nodes}
      {robot}
      <text
        x={VIEW.left + 0.4}
        y={wallV + 1.3}
        fontSize={0.9}
        fontWeight="bold"
      >
        <tspan fill={RED}>{state.red_score}</tspan>
        <tspan fill={INK}> · </tspan>
        <tspan fill={BLUE}>{state.blue_score}</tspan>
      </text>
    </svg>
  );
};

export default DeliveryMap;
