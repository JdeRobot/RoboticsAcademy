import { useState, useEffect } from "react";
import "./css/GUICanvas.css";
import { states } from "jderobot-commsmanager";
import { useExercise } from "Contexts/ExerciseContext";
import WebGUIImage from "Components/exercise/WebGUIImage";
import WebGUIContainer, {
  connectApplication,
} from "Components/exercise/WebGUIContainer";

const parseImage = (raw: string): string | undefined => {
  if (!raw) {
    return undefined;
  }
  const image = JSON.parse(raw);
  if (image.image === "" || !(image.shape instanceof Array)) {
    return undefined;
  }
  return `data:image/jpeg;base64,${image.image}`;
};

interface Score {
  harvested: number;
  green_in_crate: number;
  crushed: number;
  dropped: number;
  broken_trusses: number;
  ripe_total: number;
}

const percent = (part: number, total: number): string =>
  total > 0 ? `${Math.round((100 * part) / total)}%` : "-";

const ScoreBoard = ({ score }: { score: Score }) => {
  const ripe = score.harvested - score.green_in_crate;
  // Every tomato taken off the plant, well or badly
  const picked = score.harvested + score.crushed + score.dropped;
  return (
    <div id="harvest-score">
      <div className="main">
        Ripe harvested {ripe} / {score.ripe_total} (
        {percent(ripe, score.ripe_total)})
      </div>
      <div>Precision {percent(ripe, picked)}</div>
      <div className="penalty">
        Green {score.green_in_crate} · Crushed {score.crushed} · Dropped{" "}
        {score.dropped} · Broken trusses {score.broken_trusses}
      </div>
    </div>
  );
};

const WebGUI = () => {
  const exerciseContext = useExercise();
  const [leftImage, setLeftImage] = useState<string | undefined>(undefined);
  const [rightImage, setRightImage] = useState<string | undefined>(undefined);
  const [score, setScore] = useState<Score | undefined>(undefined);
  const [manager, setManager] = useState(exerciseContext.manager);

  useEffect(() => {
    setManager(exerciseContext.manager);
  }, [exerciseContext]);

  const updateCallback = (updateData: unknown) => {
    const update = (updateData as any).update;
    // Frequency and RTF messages go to the status bar
    if (update.image_left === undefined && update.image_right === undefined) {
      return;
    }
    if (update.score) {
      setScore(JSON.parse(update.score));
    }
    const left = parseImage(update.image_left);
    if (left) {
      setLeftImage(left);
    }
    const right = parseImage(update.image_right);
    if (right) {
      setRightImage(right);
    }
  };

  const stateCallback = (state: string) => {
    if (state === states.TOOLS_READY) {
      setLeftImage(undefined);
      setRightImage(undefined);
      setScore(undefined);
    }
  };

  connectApplication(manager, updateCallback, stateCallback);

  return (
    <WebGUIContainer>
      <WebGUIImage id="left_img" style={{ left: "0" }} src={leftImage} />
      <WebGUIImage id="right_img" style={{ left: "50%" }} src={rightImage} />
      {score && <ScoreBoard score={score} />}
    </WebGUIContainer>
  );
};

export default WebGUI;
