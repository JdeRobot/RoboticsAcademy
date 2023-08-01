import Button from '@mui/material/Button';

export const PlayCar = () => {
    const handleClick = () => {
        window.RoboticsExerciseComponents.commsManager
          .send("#gui", {
            msg: "#car3",
          })
    }
    return (
      <Button variant="outlined" onClick={handleClick} sx={{color: "blue", borderColor: "blue"}}>Play Car</Button>
    )
}