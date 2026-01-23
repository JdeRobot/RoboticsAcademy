import React, { useEffect, useState } from "react";
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
import { useNavigate } from "react-router-dom";
import { getTeaser } from "Api";

// TODO: temporary
const cache = [];

const ExerciseCard = ({
  exercise_id,
  name,
  description,
  tags,
  status,
}: Exercise) => {
  const navigate = useNavigate();
  const [teaser, setTeaser] = useState<string | undefined>(undefined);

  const navigateToExercise = (): void => {
    navigate(`/academy/studio/${exercise_id}`, { viewTransition: true });
  };

  const loadTeaser = async (id: string) => {
    // TODO: temporary
    let img = cache[id];
    if (img !== undefined) {
      setTeaser(img);
      return;
    }

    try {
      img = await getTeaser(id);
      cache[id] = img;
    } catch {
      img = FALLBACK_IMAGE;
    }
    setTeaser(img);
  };

  useEffect(() => {
    loadTeaser(exercise_id);
  }, []);

  const tagsList: string[] = JSON.parse(tags);

  return (
    <StyledExerciseCardContainer state={status}>
      <StyledExerciseCardArea onClick={() => navigateToExercise()}>
        <CardMedia
          component="img"
          style={{ flexGrow: 1 }}
          src={teaser ?? FALLBACK_IMAGE}
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
