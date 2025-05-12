import configuration from "../config.json";
import { Chip } from "@mui/material";
import CardMedia from "@mui/material/CardMedia";
import "../styles/ExerciseList.css";
import Typography from "@mui/material/Typography";
import StyleTwoToneIcon from "@mui/icons-material/StyleTwoTone";
import FALLBACK_IMAGE from "../images/monitoring2.png";

const ExerciseCard = ({ id, name, description, tags, status }) => {
  const exerciseURL = `${configuration.academy.exercises.exercise_url}`;
  const teaser = configuration.academy.exercises.teaser;
  const onMediaFallback = (event) => (event.target.src = FALLBACK_IMAGE);

  const navigateToExercise = () => {
    window.location.href = exerciseURL.interpolate({ exerciseid: id });
  };

  const tagsList = JSON.parse(tags).tags;

  return (
    <div
      className="exercise-card"
      style={{
        borderColor:
          status === "ACTIVE"
            ? "green"
            : status === "INACTIVE"
            ? "red"
            : "orange",
      }}
    >
      <div
        style={{
          flexGrow: 1,
          display: "flex",
          flexDirection: "column",
          alignItems: "flex-start",
        }}
        onClick={() => navigateToExercise()}
      >
        <CardMedia
          component="img"
          height="auto"
          style={{ flexGrow: 1 }}
          image={teaser.url.interpolate({ exerciseid: id })}
          onError={onMediaFallback}
        />
        <div className="exercise-info-container">
          <Typography
            gutterBottom
            variant="h6"
            component="div"
            color={"white"}
            sx={{ wordBreak: "break-word" }}
            style={{ alignSelf: "center", pointerEvents: "none" }}
          >
            {name}
          </Typography>
          <Typography
            id="exercise-info"
            gutterBottom
            variant="h7"
            component="div"
            color={"white"}
            style={{ pointerEvents: "none" }}
          >
            {description}
          </Typography>
          <ChipsArray tagList={tagsList} />
        </div>
      </div>
    </div>
  );
};

const ChipsArray = ({ tagList }) => {
  let chipData = tagList;
  if (!Array.isArray(chipData)) {
    chipData = chipData.split(",");
  }
  const chipsList = chipData.map((data) => (
    // eslint-disable-next-line react/jsx-key
    <Chip
      key={data}
      sx={{ m: 0.5 }}
      icon={<StyleTwoToneIcon />}
      color={"success"}
      label={data}
    />
  ));

  return <div className="exercise-tag-list">{chipsList}</div>;
};

export { ExerciseCard };
