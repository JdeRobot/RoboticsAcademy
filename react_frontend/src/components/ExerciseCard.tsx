import React from "react";
import configuration from "../config.json";
import { Chip } from "@mui/material";
import CardMedia from "@mui/material/CardMedia";
import Typography from "@mui/material/Typography";
import StyleTwoToneIcon from "@mui/icons-material/StyleTwoTone";
import FALLBACK_IMAGE from "../images/default_card.svg";
import { Exercise } from "src/types/exercises";
import {
  StyledExerciseCardArea,
  StyledExerciseCardContainer,
  StyledExerciseCardInfoContainer,
  StyledExerciseCardTagList,
} from "Styles/ExerciseCard.styles";

const ExerciseCard = ({
  exercise_id,
  name,
  description,
  tags,
  status,
}: Exercise) => {
  const exerciseURL: string = `${configuration.academy.exercises.exercise_url}`;
  const teaser = configuration.academy.exercises.teaser;

  const onMediaFallback = (event: React.SyntheticEvent<HTMLImageElement>) => {
    (event.target as HTMLImageElement).src = FALLBACK_IMAGE;
  };

  const navigateToExercise = (): void => {
    window.location.href = exerciseURL.replace("${exerciseid}", exercise_id);
  };

  const tagsList: string[] = JSON.parse(tags);
  console.log(tagsList)

  return (
    <StyledExerciseCardContainer state={status}>
      <StyledExerciseCardArea onClick={() => navigateToExercise()}>
        <CardMedia
          component="img"
          height="auto"
          style={{ flexGrow: 1 }}
          image={teaser.url.replace("${exerciseid}", exercise_id)}
          onError={onMediaFallback}
        />
        <StyledExerciseCardInfoContainer>
          <Typography
            gutterBottom
            variant="h6"
            component="div"
            color="white"
            sx={{ wordBreak: "break-word" }}
            style={{ alignSelf: "center", pointerEvents: "none" }}
          >
            {name}
          </Typography>
          <Typography
            id="exercise-info"
            gutterBottom
            variant="subtitle1"
            component="div"
            color="white"
            style={{ pointerEvents: "none" }}
          >
            {description}
          </Typography>
          <ChipsArray tagList={tagsList} />
        </StyledExerciseCardInfoContainer>
      </StyledExerciseCardArea>
    </StyledExerciseCardContainer>
  );
};

interface ChipsArrayProps {
  tagList: Array<string>;
}

const ChipsArray: React.FC<ChipsArrayProps> = ({ tagList }) => {
  let chipData = tagList;

  if (!Array.isArray(chipData)) {
    chipData = (chipData as string).split(",");
  }

  const chipsList = chipData.map((data) => (
     
    <Chip
      key={data}
      sx={{ m: 0.5 }}
      icon={<StyleTwoToneIcon />}
      color={"success"}
      label={data}
    />
  ));

  return <StyledExerciseCardTagList>{chipsList}</StyledExerciseCardTagList>;
};

export { ExerciseCard };
