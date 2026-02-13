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

const ExerciseCard = ({
  exercise_id,
  name,
  description,
  tags,
  status,
}: Exercise) => {
  const navigate = useNavigate();

  const navigateToExercise = (): void => {
    navigate(`/academy/studio/${exercise_id}`, { viewTransition: true });
  };

  const onMediaFallback = (event: React.SyntheticEvent<HTMLImageElement>) => {
    (event.target as HTMLImageElement).src = FALLBACK_IMAGE;
  };

  const tagsList: string[] = JSON.parse(tags);

  return (
    <StyledExerciseCardContainer state={status} id={exercise_id}>
      <StyledExerciseCardArea onClick={() => navigateToExercise()}>
        <CardMedia
          component="img"
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
