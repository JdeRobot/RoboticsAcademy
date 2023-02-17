import PropTypes from "prop-types";
import configuration from "../config.json";
import React, { useState } from "react";
import Card from "@mui/material/Card";
import { Box, CardActionArea, Chip } from "@mui/material";
import CardMedia from "@mui/material/CardMedia";
import "../styles/ExerciseList.css";
import CardContent from "@mui/material/CardContent";
import Typography from "@mui/material/Typography";
import ExerciseStatusIndicator from "./ExerciseStatusIndicator";
import CardActions from "@mui/material/CardActions";
import Button from "@mui/material/Button";
import PlayCircleOutlineIcon from "@mui/icons-material/PlayCircleOutline";
import InfoIcon from "@mui/icons-material/Info";
import StyleTwoToneIcon from "@mui/icons-material/StyleTwoTone";
import ArrowBackIcon from "@mui/icons-material/ArrowBack";

const Teaser = (props) => {
  const teaser_type = props.type;
  const teaser_url = props.url;

  if (teaser_type === "video") {
    return (
      <video className="card-img" autoPlay muted loop>
        <source src={teaser_url} alt={props.name} />
      </video>
    );
  } else if (teaser_type === "image") {
    return <img src={teaser_url} className={"card-img"} alt={""} />;
  }
};

Teaser.propTypes = {
  type: PropTypes.string,
  url: PropTypes.string,
  name: PropTypes.string,
};

const ExerciseCard = (props) => {
  const exerciseURL = `${configuration.academy.exercises.exercise_url}`;
  const teaser = configuration.academy.exercises.teaser;

  const navigateToExercise = () => {
    window.location.href = exerciseURL.interpolate(props);
  };
  const tagsList = JSON.parse(props.tags).tags.split(",");
  return (
    <div
      className="exercise-card"
      card-id={props.exerciseid}
      onClick={() => navigateToExercise()}
    >
      <Teaser
        type={configuration.academy.exercises.teaser.type}
        url={teaser.url.interpolate(props)}
      />
      <div className="name">{props.name}</div>
      <div className="description">{props.description}</div>
      <ChipsArray tagList={tagsList} />
    </div>
  );
};
const ExerciseCardV2 = (props) => {
  const [showDescription, setShowDescription] = useState(false);
  const exerciseURL = `${configuration.academy.exercises.exercise_url}`;
  const teaser = configuration.academy.exercises.teaser;

  const navigateToExercise = () => {
    window.location.href = exerciseURL.interpolate(props);
  };
  const tagsList = JSON.parse(props.tags).tags.split(",");

  return (
    <Card
      sx={{
        display: "flex",
        flexDirection: "column",
        justifyContent: "space-between",
      }}
      style={{ backgroundColor: "black" }}
    >
      {!showDescription && (
        <CardActionArea onClick={() => navigateToExercise()}>
          <CardMedia
            component="img"
            height="140"
            image={teaser.url.interpolate(props)}
          />
          <CardContent>
            <Typography
              gutterBottom
              variant="h6"
              component="div"
              color={"white"}
              sx={{ wordBreak: "break-word" }}
            >
              {props.name}
            </Typography>
            <ExerciseStatusIndicator status={props.status} />
          </CardContent>
        </CardActionArea>
      )}
      {showDescription && (
        <CardActionArea>
          <CardContent>
            <Typography
              gutterBottom
              variant="h6"
              component="div"
              color={"white"}
            >
              {props.description}
            </Typography>
            <ChipsArray tagList={tagsList} />
          </CardContent>
        </CardActionArea>
      )}
      <CardActions
        sx={{
          display: "flex",
          flexWrap: "wrap",
          justifyContent: "flex-start",
        }}
      >
        <Button
          variant="contained"
          color={"success"}
          startIcon={<PlayCircleOutlineIcon />}
          size="small"
          onClick={() => navigateToExercise()}
          sx={{
            m: 1,
          }}
        >
          Start
        </Button>
        <Button
          variant="contained"
          color={showDescription === true ? "error" : "secondary"}
          startIcon={
            showDescription === true ? <ArrowBackIcon /> : <InfoIcon />
          }
          size="small"
          onClick={() => {
            setShowDescription(!showDescription);
          }}
          sx={{
            m: 1,
          }}
        >
          {showDescription === true ? "Back" : "Learn More"}
        </Button>
      </CardActions>
    </Card>
  );
};

ExerciseCardV2.propTypes = {
  exerciseid: PropTypes.string,
  name: PropTypes.string,
  description: PropTypes.string,
  tags: PropTypes.string,
  status: PropTypes.string,
};

ExerciseCard.propTypes = {
  exerciseid: PropTypes.string,
  name: PropTypes.string,
  description: PropTypes.string,
  tags: PropTypes.string,
};

const ChipsArray = (props) => {
  const chipData = props.tagList;
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

  return (
    <Box
      sx={{
        display: "inline-flex",
        flexWrap: "wrap",
        p: 0.5,
        m: 0,
      }}
      component="ul"
    >
      {chipsList}
    </Box>
  );
};

ChipsArray.propTypes = {
  tagList: PropTypes.any,
};

export { ExerciseCardV2, ExerciseCard };
