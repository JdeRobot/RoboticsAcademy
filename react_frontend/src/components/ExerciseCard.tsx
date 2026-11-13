import StyleTwoToneIcon from "@mui/icons-material/StyleTwoTone";
import { Chip } from "@mui/material";
import CardMedia from "@mui/material/CardMedia";
import Typography from "@mui/material/Typography";
import React from "react";
import {
  StyledExerciseCardArea,
  StyledExerciseCardContainer,
  StyledExerciseCardInfoContainer,
  StyledExerciseCardTagList,
} from "Styles/ExerciseCard.styles";
import { Exercise } from "Types/exercises";
import FALLBACK_IMAGE from "../images/default_card.svg";

const ExerciseCard = ({
  exercise_id,
  name,
  description,
  tags,
  status,
}: Exercise) => {
  const onMediaFallback = (event: React.SyntheticEvent<HTMLImageElement>) => {
    (event.target as HTMLImageElement).src = FALLBACK_IMAGE;
  };

  const tagsList: string[] = JSON.parse(tags);

  return (
    <StyledExerciseCardContainer state={status} id={exercise_id}
      to={`/academy/studio/${exercise_id}`}
      viewTransition>
      <StyledExerciseCardArea>
        <CardMedia
          component="img"
          alt=""
          style={{ flexGrow: 1 }}
          image={`/static/${exercise_id}/teaser.png`}
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
