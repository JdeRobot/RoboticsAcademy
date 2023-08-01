import Button from '@mui/material/Button';

export const ResetCar = () => {
    const handleClick = () => {
        window.RoboticsExerciseComponents.commsManager
          .send("#gui", {
            msg: "#rst",
          })
    }
    return (
        <Button variant="outlined" onClick={handleClick} sx={{color: "blue", borderColor: "blue"}}>Reset Car</Button>
    )
}